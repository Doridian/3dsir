#include <3ds.h>
#include <dirent.h>
#include <malloc.h>
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>

#include "irenc.h"
#include "irfile.h"

#define ROOT_DIR "sdmc:/3ds/irremote"
#define IRU_MAX_TRANSFER 0x800
#define LIST_ROWS 26

static PrintConsole top, bottom;

static u8 cur_bitrate;
static bool led_state;
static IrCarrierStyle style = IrCarrierDense;
static unsigned repeats = 3;
static u8* txbuf;
static size_t txbuf_cap;
static Handle send_event;

// ---------------------------------------------------------------- sending

static Result iru_get_transfer_state(u8* out) {
    u32* cmdbuf = getThreadCommandBuffer();
    cmdbuf[0] = IPC_MakeHeader(0xF, 0, 0);
    Result rc = svcSendSyncRequest(iruGetServHandle());
    if(R_FAILED(rc)) return rc;
    *out = cmdbuf[2];
    return cmdbuf[1];
}

// What happened during the last send_chunk, for on-screen diagnostics.
static struct {
    u8 state_before; // transfer state before starting (1 = ready)
    s32 event_ms; // when the send-finished event fired, -1 if it never did
    u32 expected_ms; // on-air time of the data
    bool stuck; // state still "sending" well after the data must be out
} last_send;

static u32 ms_since(u64 t0) {
    return (svcGetSystemTick() - t0) * 1000 / SYSCLOCK_ARM11;
}

// Sends one transfer and blocks until it has gone out on air. This must not
// use IRU_WaitSendTransfer (or iruSendData with wait) up front: that command
// *cancels* the transfer in progress, which cut every signal off after ~10 ms.
static Result send_chunk(const u8* data, size_t len, u32 baud) {
    u8 state = 0;
    iru_get_transfer_state(&state);
    last_send.state_before = state;
    if(state == 2) IRU_WaitSendTransfer(); // a previous transfer never settled

    svcClearEvent(send_event);
    u64 t0 = svcGetSystemTick();
    Result rc = IRU_StartSendTransfer((u8*)data, len);
    if(R_FAILED(rc)) return rc;

    last_send.expected_ms = (u64)len * 10 * 1000 / baud;
    u32 limit_ms = last_send.expected_ms + 200;
    rc = svcWaitSynchronization(send_event, (s64)limit_ms * 1000000LL);
    last_send.event_ms = R_SUCCEEDED(rc) ? (s32)ms_since(t0) : -1;

    // Don't trust the event alone; 1 = ready, 2 = sending. The UART cannot be
    // slower than its bit rate, so once the limit has passed all data is out
    // and the transfer is only stuck in the sysmodule's bookkeeping: cancel it.
    last_send.stuck = false;
    while(R_SUCCEEDED(iru_get_transfer_state(&state)) && state == 2) {
        if(ms_since(t0) > limit_ms) {
            last_send.stuck = true;
            IRU_WaitSendTransfer();
            break;
        }
        svcSleepThread(1000000LL);
    }
    return 0;
}

// Splits a long byte stream into ir:u sized transfers. Each split adds ~10 ms
// of silence, so cut in the middle of the longest space (run of space_byte)
// where the extra delay only stretches a gap the receiver already ignores.
static Result send_bytes(const u8* data, size_t n, u32 baud, u8 space_byte) {
    size_t off = 0;
    while(off < n) {
        size_t len = n - off;
        if(len > IRU_MAX_TRANSFER) {
            size_t best_start = 0, best_len = 0, run = 0;
            for(size_t i = 1; i <= IRU_MAX_TRANSFER; i++) {
                run = data[off + i - 1] == space_byte ? run + 1 : 0;
                if(run > best_len) {
                    best_len = run;
                    best_start = i - run;
                }
            }
            len = best_len ? best_start + best_len / 2 + 1 : IRU_MAX_TRANSFER;
        }
        Result rc = send_chunk(data + off, len, baud);
        if(R_FAILED(rc)) return rc;
        off += len;
    }
    return 0;
}

static Result set_carrier(const IrCarrier* c) {
    if(c->iru_value == cur_bitrate) return 0;
    Result rc = IRU_SetBitRate(c->iru_value);
    if(R_SUCCEEDED(rc)) cur_bitrate = c->iru_value;
    return rc;
}

static Result send_timings(const IrTimings* tm) {
    const IrCarrier* c = ir_pick_carrier(tm->freq, style);
    Result rc = set_carrier(c);
    if(R_FAILED(rc)) return rc;

    size_t n = ir_render_uart(tm, c, NULL, 0);
    if(n > txbuf_cap) {
        free(txbuf);
        txbuf_cap = n;
        txbuf = malloc(n);
    }
    ir_render_uart(tm, c, txbuf, n);
    return send_bytes(txbuf, n, c->baud, ir_space_byte(c));
}

static void carrier_test(void) {
    static u8 carrier[IRU_MAX_TRANSFER];
    const IrCarrier* c = ir_pick_carrier(38000, style);
    memset(carrier, c->cycle_slots == 2 ? 0x55 : 0x00, sizeof(carrier));
    consoleSelect(&bottom);
    printf("Carrier test %lu Hz: hold A (B to exit)\n", c->baud / c->cycle_slots);
    printf("Look at the IR port through a phone camera.\n");
    Result rc = set_carrier(c);
    if(R_FAILED(rc)) printf("SetBitRate: %08lX\n", rc);

    while(aptMainLoop()) {
        hidScanInput();
        u32 held = hidKeysHeld();
        if(hidKeysDown() & KEY_B) break;
        if(held & KEY_A) {
            rc = send_bytes(carrier, sizeof(carrier), c->baud, ir_space_byte(c));
            if(R_FAILED(rc)) {
                printf("send: %08lX\n", rc);
                svcSleepThread(500000000LL);
            }
        } else {
            gspWaitForVBlank();
        }
    }
}

// Sends a fixed amount of data at every bit rate and times it. If ir:u keeps
// the UART busy the whole time, the measured rate matches the nominal one; a
// lower rate means gaps between bytes (or that SetBitRate did not take).
static void timing_check(void) {
    static const u32 nominal[19] = {[3] = 115200, 96000, 72000, 48000, 36000, 24000, 18000, 12000,
                                    9600, 6000, 3000, 57600, 38400, 19200, 7200, 4800};
    static u8 data[IRU_MAX_TRANSFER];
    consoleSelect(&bottom);
    consoleClear();
    printf("val nominal  got   ms  measured\n");
    for(u8 v = 3; v <= 18 && aptMainLoop(); v++) {
        size_t n = nominal[v] >= 36000 ? sizeof(data) : 256;
        Result rc = IRU_SetBitRate(v);
        u8 got = 0;
        IRU_GetBitRate(&got);
        u64 t0 = svcGetSystemTick();
        if(R_SUCCEEDED(rc)) rc = send_chunk(data, n, nominal[v]);
        u64 ticks = svcGetSystemTick() - t0;
        if(R_FAILED(rc)) {
            printf("%3u %7lu %4u  err %08lX\n", v, nominal[v], got, rc);
            continue;
        }
        u32 us = ticks * 1000000 / SYSCLOCK_ARM11;
        printf("%3u %7lu %4u %4lu %8llu\n", v, nominal[v], got, us / 1000,
               (u64)n * 10 * 1000000 / (us ? us : 1));
    }
    cur_bitrate = 0; // force a SetBitRate on the next send
    printf("B to go back\n");
    while(aptMainLoop()) {
        gspWaitForVBlank();
        hidScanInput();
        if(hidKeysDown() & KEY_B) break;
    }
}

// ---------------------------------------------------------------- lists

static void adjust_repeats(u32 down) {
    unsigned old = repeats;
    if((down & KEY_L) && repeats > 1) repeats--;
    if((down & KEY_R) && repeats < 10) repeats++;
    if(repeats != old) {
        consoleSelect(&bottom);
        printf("Repeats: %u\n", repeats);
    }
}

static void draw_list(const char* title, char** items, size_t n, size_t sel, const char* help) {
    consoleSelect(&top);
    consoleClear();
    printf("\x1b[1;1H\x1b[36m%.49s\x1b[0m\n", title);
    size_t first = sel >= LIST_ROWS / 2 ? sel - LIST_ROWS / 2 : 0;
    if(n > LIST_ROWS && first > n - LIST_ROWS) first = n - LIST_ROWS;
    for(size_t i = first; i < n && i < first + LIST_ROWS; i++)
        printf("%s%c %.46s\x1b[0m\n", i == sel ? "\x1b[33m" : "", i == sel ? '>' : ' ', items[i]);
    printf("\x1b[29;1H\x1b[36m%.49s\x1b[0m", help);
}

static int move_sel(u32 down, size_t n, size_t* sel) {
    if(!n) return 0;
    size_t old = *sel;
    if(down & KEY_DOWN) *sel = (*sel + 1) % n;
    if(down & KEY_UP) *sel = (*sel + n - 1) % n;
    if(down & KEY_RIGHT) *sel = *sel + 10 < n ? *sel + 10 : n - 1;
    if(down & KEY_LEFT) *sel = *sel >= 10 ? *sel - 10 : 0;
    return old != *sel;
}

static void report(const IrSignal* s, Result rc, const IrTimings* tm) {
    const IrCarrier* c = ir_pick_carrier(tm->freq, style);
    consoleSelect(&bottom);
    if(s->raw)
        printf("%.20s raw %luHz", s->name, s->frequency);
    else
        printf("%.20s %s %lX/%lX", s->name, s->protocol, s->address, s->command);
    if(R_FAILED(rc))
        printf(" \x1b[31m%08lX\x1b[0m\n", rc);
    else
        printf(" %uB @%lu\n", ir_render_uart(tm, c, NULL, 0), c->baud);
    if(R_SUCCEEDED(rc))
        printf("  pre %u ev %ld/%lu ms%s\n", last_send.state_before, last_send.event_ms,
               last_send.expected_ms, last_send.stuck ? " \x1b[33mstuck\x1b[0m" : "");
}

// Sends every signal named like `name` (e.g. all "Power" codes in a universal
// library), one after another, until done or B is pressed.
static void sweep(IrFile* f, const char* name, IrTimings* tm) {
    consoleSelect(&bottom);
    printf("\x1b[32mSweeping \"%s\", B to stop\x1b[0m\n", name);
    size_t count = 0;
    for(size_t i = 0; i < f->n && aptMainLoop(); i++) {
        hidScanInput();
        if(hidKeysDown() & KEY_B) {
            printf("stopped\n");
            return;
        }
        if(strcmp(f->sig[i].name, name)) continue;
        if(!irfile_signal_timings(&f->sig[i], repeats, tm)) continue;
        Result rc = send_timings(tm);
        report(&f->sig[i], rc, tm);
        count++;
        svcSleepThread(150000000LL); // let the TV settle between codes
    }
    printf("sent %zu codes\n", count);
}

static void signal_menu(IrFile* f, const char* title) {
    char** items = malloc(f->n * sizeof(char*));
    for(size_t i = 0; i < f->n; i++) items[i] = f->sig[i].name;
    size_t sel = 0;
    IrTimings tm;
    ir_timings_init(&tm);
    draw_list(title, items, f->n, sel, "A send  Y sweep same name  B back");

    while(aptMainLoop()) {
        gspWaitForVBlank();
        hidScanInput();
        u32 down = hidKeysDown();
        if(down & KEY_B) break;
        if(move_sel(down, f->n, &sel)) draw_list(title, items, f->n, sel, "A send  Y sweep same name  B back");
        adjust_repeats(down);
        if(!f->n) continue;

        if(down & KEY_A) {
            IrSignal* s = &f->sig[sel];
            if(!irfile_signal_timings(s, repeats, &tm)) {
                consoleSelect(&bottom);
                printf("unsupported: %s\n", s->raw ? "raw" : s->protocol);
                continue;
            }
            // Keep resending the same waveform (same RC5/RC6 toggle bit) while held.
            do {
                Result rc = send_timings(&tm);
                report(s, rc, &tm);
                if(R_FAILED(rc)) break;
                svcSleepThread(40000000LL);
                hidScanInput();
            } while(hidKeysHeld() & KEY_A);
        }
        if(down & KEY_Y) {
            sweep(f, f->sig[sel].name, &tm);
            draw_list(title, items, f->n, sel, "A send  Y sweep same name  B back");
        }
    }
    ir_timings_free(&tm);
    free(items);
}

// ---------------------------------------------------------------- built-ins

static const struct {
    const char* name;
    const char* protocol;
    u32 address, command;
} builtin[] = {
    {"Samsung Vol+", "Samsung32", 0x07, 0x07},
    {"Samsung Power", "Samsung32", 0x07, 0x02},
    {"LG Vol+", "NEC", 0x04, 0x02},
    {"LG Power", "NEC", 0x04, 0x08},
    {"Sony Vol+", "SIRC", 0x01, 0x12},
    {"Sony Power", "SIRC", 0x01, 0x15},
    {"Philips Vol+", "RC5", 0x00, 0x10},
    {"Philips Power", "RC5", 0x00, 0x0C},
};

static void builtin_menu(void) {
    IrFile f;
    f.n = sizeof(builtin) / sizeof(builtin[0]);
    f.sig = calloc(f.n, sizeof(IrSignal));
    for(size_t i = 0; i < f.n; i++) {
        snprintf(f.sig[i].name, sizeof(f.sig[i].name), "%s", builtin[i].name);
        snprintf(f.sig[i].protocol, sizeof(f.sig[i].protocol), "%s", builtin[i].protocol);
        f.sig[i].address = builtin[i].address;
        f.sig[i].command = builtin[i].command;
    }
    signal_menu(&f, "Built-in test codes");
    irfile_free(&f);
}

// ---------------------------------------------------------------- browser

static int cmp_str(const void* a, const void* b) {
    return strcasecmp(*(char* const*)a, *(char* const*)b);
}

static size_t list_dir(const char* path, char*** out) {
    size_t n = 0, cap = 16;
    char** items = malloc(cap * sizeof(char*));
    DIR* d = opendir(path);
    struct dirent* e;
    while(d && (e = readdir(d))) {
        if(e->d_name[0] == '.') continue;
        bool dir = e->d_type == DT_DIR;
        size_t len = strlen(e->d_name);
        if(!dir && (len < 3 || strcasecmp(e->d_name + len - 3, ".ir"))) continue;
        if(n == cap) items = realloc(items, (cap *= 2) * sizeof(char*));
        items[n] = malloc(len + 2);
        sprintf(items[n++], "%s%s", e->d_name, dir ? "/" : "");
    }
    if(d) closedir(d);
    qsort(items, n, sizeof(char*), cmp_str);
    *out = items;
    return n;
}

static void browse(const char* path, bool is_root) {
    char** names;
    size_t n = list_dir(path, &names);
    size_t extra = is_root ? 3 : 0;
    char** items = malloc((n + extra) * sizeof(char*));
    if(is_root) {
        items[0] = "[Built-in test codes]";
        items[1] = "[Carrier test]";
        items[2] = "[Bit rate / timing check]";
    }
    memcpy(items + extra, names, n * sizeof(char*));
    size_t total = n + extra, sel = 0;
    const char* help = is_root ? "A open X IR LED SELECT carrier START exit" : "A open  B back";
    draw_list(path, items, total, sel, help);

    while(aptMainLoop()) {
        gspWaitForVBlank();
        hidScanInput();
        u32 down = hidKeysDown();
        if(is_root && (down & KEY_START)) break;
        if(!is_root && (down & KEY_B)) break;
        if(move_sel(down, total, &sel)) draw_list(path, items, total, sel, help);

        adjust_repeats(down);
        if(down & KEY_SELECT) {
            style = style == IrCarrierClean ? IrCarrierDense : IrCarrierClean;
            consoleSelect(&bottom);
            printf("Carrier style: %s\n", style == IrCarrierClean ? "clean (0x55, 36/48/57.6k)" : "dense (0x00, 36/38.4k)");
        }
        if(down & KEY_X) {
            led_state = !led_state;
            Result rc = IRU_SetIRLEDState(led_state);
            consoleSelect(&bottom);
            printf("SetIRLEDState(%d): %08lX\n", led_state, rc);
        }
        if(!(down & KEY_A) || !total) continue;

        if(is_root && sel == 0) {
            builtin_menu();
        } else if(is_root && sel == 1) {
            carrier_test();
        } else if(is_root && sel == 2) {
            timing_check();
        } else {
            char sub[512];
            const char* name = items[sel];
            snprintf(sub, sizeof(sub), "%s/%s", path, name);
            size_t len = strlen(sub);
            if(sub[len - 1] == '/') {
                sub[len - 1] = 0;
                browse(sub, false);
            } else {
                IrFile f;
                consoleSelect(&bottom);
                if(irfile_load(sub, &f)) {
                    printf("%s: %zu signals\n", name, f.n);
                    signal_menu(&f, name);
                    irfile_free(&f);
                } else {
                    printf("cannot open %s\n", sub);
                }
            }
        }
        draw_list(path, items, total, sel, help);
    }

    for(size_t i = 0; i < n; i++) free(names[i]);
    free(names);
    free(items);
}

int main(void) {
    gfxInitDefault();
    consoleInit(GFX_TOP, &top);
    consoleInit(GFX_BOTTOM, &bottom);
    consoleSelect(&bottom);

    u32* iru_shm = memalign(0x1000, 0x1000);
    Result rc = iruInit(iru_shm, 0x1000);
    printf("iruInit: %08lX\n", rc);
    if(R_SUCCEEDED(rc)) {
        IRU_GetBitRate(&cur_bitrate);
        rc = IRU_GetSendFinishedEvent(&send_event);
        printf("GetSendFinishedEvent: %08lX\n", rc);
        printf("default bitrate value: %u\n", cur_bitrate);
        printf("Carrier style: dense (SELECT toggles)\n");
        printf("Repeats: %u (L/R adjust)\n", repeats);
        mkdir(ROOT_DIR, 0777);
        printf("Put Flipper .ir files in\n" ROOT_DIR "/\n");
        browse(ROOT_DIR, true);
        svcCloseHandle(send_event);
        iruExit();
    } else {
        printf("ir:u unavailable. START to exit.\n");
        while(aptMainLoop()) {
            gspWaitForVBlank();
            hidScanInput();
            if(hidKeysDown() & KEY_START) break;
        }
    }

    free(txbuf);
    free(iru_shm);
    gfxExit();
    return 0;
}
