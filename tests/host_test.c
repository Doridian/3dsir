// Host-side checks for the encoder: renders signals to UART bytes, simulates
// what the SC16IS750 IrDA encoder emits and what a TSOP-style receiver would
// demodulate, then compares against the intended waveform.
//
//   cc -O2 -Isource tests/host_test.c source/irenc.c source/irfile.c -o host_test
//   ./host_test path/to/tv.ir

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "irenc.h"
#include "irfile.h"

// Light pulses (slot indices) that the IrDA encoder emits for a byte stream.
static size_t pulses(const uint8_t* bytes, size_t n, uint64_t* out) {
    size_t k = 0;
    for(size_t f = 0; f < n; f++) {
        out[k++] = f * 10; // start bit
        for(int b = 0; b < 8; b++)
            if(!(bytes[f] >> b & 1)) out[k++] = f * 10 + 1 + b;
    }
    return k;
}

// Crude receiver model: a burst of >= 4 pulses no more than 2 slots apart is a
// mark; anything else (like the lone start bits in spaces) is ignored.
static void demod(const uint64_t* p, size_t np, uint32_t baud, IrTimings* out) {
    ir_timings_clear(out);
    uint64_t prev_end = 0;
    size_t i = 0;
    while(i < np) {
        size_t j = i;
        while(j + 1 < np && p[j + 1] - p[j] <= 2) j++;
        if(j - i + 1 >= 4) {
            uint64_t start = p[i], end = p[j] + 1;
            if(out->n) ir_timings_space(out, (uint32_t)((start - prev_end) * 1000000 / baud));
            ir_timings_mark(out, (uint32_t)((end - start) * 1000000 / baud));
            prev_end = end;
        }
        i = j + 1;
    }
}

static uint8_t buf[1 << 17];
static uint64_t pbuf[(1 << 17) * 9];

static IrCarrierStyle style;

static int check(const char* label, const IrTimings* want, IrTimings* got, uint32_t* max_err) {
    const IrCarrier* c = ir_pick_carrier(want->freq, style);
    size_t n = ir_render_uart(want, c, buf, sizeof(buf));
    if(n > sizeof(buf)) {
        printf("%s: too long (%zu bytes)\n", label, n);
        return 1;
    }
    demod(pbuf, pulses(buf, n, pbuf), c->baud, got);
    if(got->n != want->n) {
        printf("%s: %zu segments, expected %zu\n", label, got->n, want->n);
        return 1;
    }
    for(size_t i = 0; i < want->n; i++) {
        uint32_t e = got->t[i] > want->t[i] ? got->t[i] - want->t[i] : want->t[i] - got->t[i];
        if(e > *max_err) *max_err = e;
        // The frame structure (fixed start pulse, fixed stop gap) can move an
        // edge by ~2 slots. TSOP-style receivers themselves only promise output
        // pulse widths within about +-5 carrier cycles, so allow that much.
        if(e > 5 * 1000000 * c->cycle_slots / c->baud) {
            printf("%s: segment %zu is %u us, expected %u us\n", label, i, got->t[i], want->t[i]);
            return 1;
        }
    }
    return 0;
}

// Reads back a pulse-distance frame (32 bits after the leader) as sent on air.
static uint32_t pdm_bits(const IrTimings* t, uint32_t threshold) {
    uint32_t v = 0;
    for(int i = 0; i < 32; i++) v = v << 1 | (t->t[3 + 2 * i] > threshold);
    return v;
}

static int run(int argc, char** argv) {
    int fails = 0;
    IrTimings want, got;
    ir_timings_init(&want);
    ir_timings_init(&got);
    uint32_t max_err = 0;

    // Well-known codes, written in on-air bit order.
    struct {
        const char* proto;
        uint32_t addr, cmd, expect;
    } known[] = {
        {"NEC", 0x04, 0x08, 0x20DF10EF}, // LG power
        {"Samsung32", 0x07, 0x02, 0xE0E040BF}, // Samsung power
    };
    for(size_t i = 0; i < sizeof(known) / sizeof(known[0]); i++) {
        ir_encode_parsed(known[i].proto, known[i].addr, known[i].cmd, &want);
        fails += check(known[i].proto, &want, &got, &max_err);
        uint32_t v = pdm_bits(&got, 1000);
        if(v != known[i].expect) {
            printf("%s: got %08X expected %08X\n", known[i].proto, v, known[i].expect);
            fails++;
        }
    }

    // Sony power (address 1, command 0x15) is 0xA90 on air.
    ir_encode_parsed("SIRC", 1, 0x15, &want);
    fails += check("SIRC", &want, &got, &max_err);
    uint32_t sirc = 0;
    for(int i = 0; i < 12; i++) sirc = sirc << 1 | (got.t[2 + 2 * i] > 900);
    if(sirc != 0xA90) {
        printf("SIRC: got %03X expected A90\n", sirc);
        fails++;
    }

    if(argc > 1) {
        IrFile f;
        if(!irfile_load(argv[1], &f)) {
            printf("cannot load %s\n", argv[1]);
            return 1;
        }
        size_t ok = 0, unsupported = 0, longest = 0;
        for(size_t i = 0; i < f.n; i++) {
            char label[96];
            snprintf(label, sizeof(label), "#%zu %s (%s)", i, f.sig[i].name,
                     f.sig[i].raw ? "raw" : f.sig[i].protocol);
            if(!irfile_signal_timings(&f.sig[i], 3, &want)) {
                printf("%s: unsupported\n", label);
                unsupported++;
                continue;
            }
            size_t n = ir_render_uart(&want, ir_pick_carrier(want.freq, style), NULL, 0);
            if(n > longest) longest = n;
            if(check(label, &want, &got, &max_err))
                fails++;
            else
                ok++;
        }
        printf("%s: %zu signals, %zu ok, %zu unsupported, longest %zu bytes\n", argv[1], f.n, ok,
               unsupported, longest);
        irfile_free(&f);
    }

    printf("max timing error %u us, %d failures\n", max_err, fails);
    ir_timings_free(&want);
    ir_timings_free(&got);
    return fails;
}

int main(int argc, char** argv) {
    printf("== dense carrier\n");
    style = IrCarrierDense;
    int fails = run(argc, argv);
    printf("== clean carrier\n");
    style = IrCarrierClean;
    fails += run(argc, argv);
    return fails != 0;
}
