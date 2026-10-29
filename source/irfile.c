#include "irfile.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char* trim(char* s) {
    while(isspace((unsigned char)*s)) s++;
    char* e = s + strlen(s);
    while(e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}

// "04 00 00 00" -> 0x00000004 (little endian byte list)
static uint32_t parse_le_hex(const char* s) {
    uint32_t v = 0;
    for(int i = 0; i < 4; i++) {
        char* end;
        unsigned long b = strtoul(s, &end, 16);
        if(end == s) break;
        v |= (uint32_t)(b & 0xFF) << (8 * i);
        s = end;
    }
    return v;
}

static void parse_data(IrSignal* sig, const char* s) {
    size_t cap = 64;
    sig->data = malloc(cap * sizeof(uint32_t));
    sig->data_n = 0;
    for(;;) {
        char* end;
        unsigned long v = strtoul(s, &end, 10);
        if(end == s) break;
        if(sig->data_n == cap) {
            cap *= 2;
            sig->data = realloc(sig->data, cap * sizeof(uint32_t));
        }
        sig->data[sig->data_n++] = v;
        s = end;
    }
}

bool irfile_load(const char* path, IrFile* out) {
    FILE* fp = fopen(path, "r");
    if(!fp) return false;

    memset(out, 0, sizeof(*out));
    size_t cap = 0;
    IrSignal* cur = NULL;
    static char line[16384];

    while(fgets(line, sizeof(line), fp)) {
        char* colon = strchr(line, ':');
        if(line[0] == '#' || !colon) continue;
        *colon = 0;
        char* key = trim(line);
        char* val = trim(colon + 1);

        if(!strcmp(key, "name")) {
            if(out->n == cap) {
                cap = cap ? cap * 2 : 32;
                out->sig = realloc(out->sig, cap * sizeof(IrSignal));
            }
            cur = &out->sig[out->n++];
            memset(cur, 0, sizeof(*cur));
            snprintf(cur->name, sizeof(cur->name), "%s", val);
        } else if(!cur) {
            continue;
        } else if(!strcmp(key, "type")) {
            cur->raw = !strcmp(val, "raw");
        } else if(!strcmp(key, "protocol")) {
            snprintf(cur->protocol, sizeof(cur->protocol), "%s", val);
        } else if(!strcmp(key, "address")) {
            cur->address = parse_le_hex(val);
        } else if(!strcmp(key, "command")) {
            cur->command = parse_le_hex(val);
        } else if(!strcmp(key, "frequency")) {
            cur->frequency = strtoul(val, NULL, 10);
        } else if(!strcmp(key, "data")) {
            free(cur->data);
            parse_data(cur, val);
        }
    }
    fclose(fp);
    return true;
}

void irfile_free(IrFile* f) {
    for(size_t i = 0; i < f->n; i++) free(f->sig[i].data);
    free(f->sig);
    f->sig = NULL;
    f->n = 0;
}

// Gap between repeats when the signal does not carry its own trailing space.
#define DEFAULT_REPEAT_GAP_US 40000

bool irfile_signal_timings(const IrSignal* s, unsigned repeats, IrTimings* out) {
    uint32_t gap = DEFAULT_REPEAT_GAP_US;
    if(!s->raw) {
        if(!ir_encode_parsed(s->protocol, s->address, s->command, out)) return false;
    } else {
        ir_timings_clear(out);
        if(!s->data_n) return false;
        out->freq = s->frequency;
        for(size_t i = 0; i < s->data_n; i++) {
            if(i % 2 == 0)
                ir_timings_mark(out, s->data[i]);
            else
                ir_timings_space(out, s->data[i]);
        }
        // A raw capture's trailing space is the remote's own repeat gap.
        if(out->n % 2 == 0) gap = out->t[--out->n];
    }
    ir_timings_repeat(out, repeats, gap);
    return true;
}
