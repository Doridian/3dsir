#include "irenc.h"

#include <stdlib.h>
#include <string.h>

// Protocol encodings follow the Flipper Zero firmware (lib/infrared) so that
// Flipper .ir files and the Flipper-IRDB work unchanged.

static const IrCarrier dense_carriers[] = {
    {36000, 7, 1},
    {38400, 15, 1},
    {48000, 6, 1},
    {57600, 14, 1},
};

static const IrCarrier clean_carriers[] = {
    {72000, 5, 2},
    {96000, 4, 2},
    {115200, 3, 2},
};

static bool rc5_toggle;
static bool rc6_toggle;

void ir_timings_init(IrTimings* tm) {
    memset(tm, 0, sizeof(*tm));
}

void ir_timings_free(IrTimings* tm) {
    free(tm->t);
    ir_timings_init(tm);
}

void ir_timings_clear(IrTimings* tm) {
    tm->n = 0;
    tm->freq = 0;
}

static void push(IrTimings* tm, bool mark, uint32_t us) {
    if(!us) return;
    // Waveforms always start with a mark; leading silence is meaningless.
    if(!tm->n && !mark) return;
    bool last_is_mark = (tm->n % 2) == 1;
    if(tm->n && last_is_mark == mark) {
        tm->t[tm->n - 1] += us;
        return;
    }
    if(tm->n == tm->cap) {
        tm->cap = tm->cap ? tm->cap * 2 : 128;
        tm->t = realloc(tm->t, tm->cap * sizeof(*tm->t));
    }
    tm->t[tm->n++] = us;
}

void ir_timings_mark(IrTimings* tm, uint32_t us) {
    push(tm, true, us);
}

void ir_timings_space(IrTimings* tm, uint32_t us) {
    push(tm, false, us);
}

void ir_timings_repeat(IrTimings* tm, unsigned times, uint32_t gap_us) {
    size_t n = tm->n;
    for(unsigned r = 1; r < times; r++) {
        ir_timings_space(tm, gap_us);
        for(size_t i = 0; i < n; i++) push(tm, i % 2 == 0, tm->t[i]);
    }
}

static uint32_t total_us(const IrTimings* tm, size_t from) {
    uint32_t sum = 0;
    for(size_t i = from; i < tm->n; i++) sum += tm->t[i];
    return sum;
}

typedef struct {
    uint16_t preamble_mark, preamble_space;
    uint16_t bit1_mark, bit1_space;
    uint16_t bit0_mark, bit0_space;
} Pdwm;

// Pulse distance/width modulation, LSB first, as in infrared_common_encode_pdwm.
static void
    pdwm(IrTimings* o, const Pdwm* p, const uint8_t* data, int bits, bool stop_mark) {
    if(p->preamble_mark) {
        ir_timings_mark(o, p->preamble_mark);
        ir_timings_space(o, p->preamble_space);
    }
    for(int i = 0; i < bits; i++) {
        bool bit = (data[i / 8] >> (i % 8)) & 1;
        ir_timings_mark(o, bit ? p->bit1_mark : p->bit0_mark);
        ir_timings_space(o, bit ? p->bit1_space : p->bit0_space);
    }
    if(stop_mark) ir_timings_mark(o, p->bit1_mark);
}

// Manchester, MSB first. RC5 sends a 1 as space-then-mark, RC6 as mark-then-space.
static void manchester(IrTimings* o, uint32_t value, int bits, uint32_t half, bool one_is_mark_first) {
    for(int i = bits - 1; i >= 0; i--) {
        bool bit = (value >> i) & 1;
        if(bit == one_is_mark_first) {
            ir_timings_mark(o, half);
            ir_timings_space(o, half);
        } else {
            ir_timings_space(o, half);
            ir_timings_mark(o, half);
        }
    }
}

static void put_le(uint8_t* d, uint64_t v, int bytes) {
    for(int i = 0; i < bytes; i++) d[i] = (uint8_t)(v >> (8 * i));
}

static const Pdwm nec_t = {9000, 4500, 560, 1690, 560, 560};
static const Pdwm samsung_t = {4500, 4500, 550, 1650, 550, 550};
static const Pdwm sirc_t = {2400, 600, 1200, 600, 600, 600};
static const Pdwm kaseikyo_t = {3456, 1728, 432, 1296, 432, 432};
static const Pdwm pioneer_t = {8500, 4225, 500, 1500, 500, 500};
static const Pdwm rca_t = {4000, 4000, 500, 2000, 500, 1000};

bool ir_encode_parsed(const char* protocol, uint32_t address, uint32_t command, IrTimings* out) {
    uint8_t d[8] = {0};
    ir_timings_clear(out);
    out->freq = 38000;

    if(!strcmp(protocol, "NEC")) {
        uint8_t a = address, c = command;
        put_le(d, a | (uint32_t)(uint8_t)~a << 8 | (uint32_t)c << 16 | (uint32_t)(uint8_t)~c << 24, 4);
        pdwm(out, &nec_t, d, 32, true);
    } else if(!strcmp(protocol, "NECext")) {
        put_le(d, (address & 0xFFFF) | (command & 0xFFFF) << 16, 4);
        pdwm(out, &nec_t, d, 32, true);
    } else if(!strcmp(protocol, "NEC42")) {
        uint64_t v = (address & 0x1FFF) | (uint64_t)(~address & 0x1FFF) << 13 |
                     (uint64_t)(command & 0xFF) << 26 | (uint64_t)(~command & 0xFF) << 34;
        put_le(d, v, 6);
        pdwm(out, &nec_t, d, 42, true);
    } else if(!strcmp(protocol, "NEC42ext")) {
        uint64_t v = (address & 0x3FFFFFF) | (uint64_t)(command & 0xFFFF) << 26;
        put_le(d, v, 6);
        pdwm(out, &nec_t, d, 42, true);
    } else if(!strcmp(protocol, "Samsung32")) {
        uint8_t a = address, c = command;
        put_le(d, a | (uint32_t)a << 8 | (uint32_t)c << 16 | (uint32_t)(uint8_t)~c << 24, 4);
        pdwm(out, &samsung_t, d, 32, true);
    } else if(!strcmp(protocol, "SIRC") || !strcmp(protocol, "SIRC15") ||
              !strcmp(protocol, "SIRC20")) {
        int bits = protocol[4] == '1' ? 15 : protocol[4] == '2' ? 20 : 12;
        uint32_t amask = (1u << (bits - 7)) - 1;
        put_le(d, (command & 0x7F) | (address & amask) << 7, 4);
        out->freq = 40000;
        // Sony receivers need at least three frames, 45 ms start to start.
        for(int r = 0; r < 3; r++) {
            size_t start = out->n;
            pdwm(out, &sirc_t, d, bits, false);
            if(r < 2) {
                uint32_t frame = total_us(out, start);
                ir_timings_space(out, frame < 45000 ? 45000 - frame : 10000);
            }
        }
    } else if(!strcmp(protocol, "RC5") || !strcmp(protocol, "RC5X")) {
        // S1, S2 (cleared for RC5X, i.e. command bit 6), toggle, 5 address, 6 command.
        uint32_t v = 1u << 13 | (protocol[3] ? 0 : 1u << 12) | (rc5_toggle ? 1u << 11 : 0) |
                     (address & 0x1F) << 6 | (command & 0x3F);
        rc5_toggle = !rc5_toggle;
        out->freq = 36000;
        manchester(out, v, 14, 888, false);
    } else if(!strcmp(protocol, "RC6")) {
        out->freq = 36000;
        ir_timings_mark(out, 2666);
        ir_timings_space(out, 889);
        manchester(out, 0x8, 4, 444, true); // start bit 1, mode 000
        manchester(out, rc6_toggle, 1, 888, true); // toggle bit is double width
        manchester(out, (address & 0xFF) << 8 | (command & 0xFF), 16, 444, true);
        rc6_toggle = !rc6_toggle;
    } else if(!strcmp(protocol, "Kaseikyo")) {
        uint8_t id = (address >> 24) & 3;
        uint16_t vendor = (address >> 8) & 0xFFFF;
        uint8_t genre1 = (address >> 4) & 0xF, genre2 = address & 0xF;
        d[0] = vendor & 0xFF;
        d[1] = vendor >> 8;
        uint8_t parity = d[0] ^ d[1];
        parity = (parity & 0xF) ^ (parity >> 4);
        d[2] = (parity & 0xF) | genre1 << 4;
        d[3] = genre2 | (command & 0xF) << 4;
        d[4] = id << 6 | ((command >> 4) & 0x3F);
        d[5] = d[2] ^ d[3] ^ d[4];
        pdwm(out, &kaseikyo_t, d, 48, true);
    } else if(!strcmp(protocol, "Pioneer")) {
        uint8_t a = address, c = command;
        put_le(d, a | (uint32_t)(uint8_t)~a << 8 | (uint32_t)c << 16 | (uint32_t)(uint8_t)~c << 24, 4);
        out->freq = 40000;
        // The 33rd (zero) bit doubles as the stop mark; sent twice, 26 ms apart.
        pdwm(out, &pioneer_t, d, 33, false);
        ir_timings_space(out, 26000);
        pdwm(out, &pioneer_t, d, 33, false);
    } else if(!strcmp(protocol, "RCA")) {
        uint8_t a = address, c = command;
        put_le(d, (a & 0xF) | (uint32_t)c << 4 | (uint32_t)(~a & 0xF) << 12 | (uint32_t)(uint8_t)~c << 16, 3);
        // Flipper drives RCA at 38 kHz, but real RCA receivers are 56 kHz parts.
        out->freq = 56000;
        pdwm(out, &rca_t, d, 24, true);
    } else {
        return false;
    }

    // Drop the trailing space; the line idles dark anyway.
    if(out->n % 2 == 0 && out->n) out->n--;
    return true;
}

static uint32_t freq_diff(const IrCarrier* c, uint32_t freq) {
    uint32_t f = c->baud / c->cycle_slots;
    return f > freq ? f - freq : freq - f;
}

const IrCarrier* ir_pick_carrier(uint32_t freq, IrCarrierStyle style) {
    const IrCarrier* list = style == IrCarrierClean ? clean_carriers : dense_carriers;
    size_t n = style == IrCarrierClean ? sizeof(clean_carriers) / sizeof(clean_carriers[0]) :
                                         sizeof(dense_carriers) / sizeof(dense_carriers[0]);
    if(!freq) freq = 38000;
    const IrCarrier* best = &list[0];
    for(size_t i = 1; i < n; i++)
        if(freq_diff(&list[i], freq) < freq_diff(best, freq)) best = &list[i];
    return best;
}

uint8_t ir_space_byte(const IrCarrier* c) {
    return c->cycle_slots == 2 ? 0xE6 : 0xFF;
}

static uint64_t us_to_slot(uint64_t us, uint32_t baud) {
    return (us * baud + 500000) / 1000000;
}

size_t ir_render_uart(const IrTimings* tm, const IrCarrier* c, uint8_t* out, size_t out_max) {
    uint32_t baud = c->baud;
    uint64_t total_slots = us_to_slot(total_us(tm, 0), baud);
    size_t frames = (total_slots + 9) / 10;

    // Bit slot k (at 1/baud resolution) is lit iff it falls inside a mark.
    // Each UART frame covers 10 slots: start (always lit), 8 data, stop (always dark).
    size_t seg = 0;
    uint64_t seg_end_us = tm->n ? tm->t[0] : 0;
    uint64_t seg_end_slot = us_to_slot(seg_end_us, baud);

    for(size_t f = 0; f < frames; f++) {
        uint8_t byte = 0xFF;
        bool in_mark = false;
        for(int b = 0; b < 10; b++) {
            uint64_t slot = f * 10 + b;
            while(seg < tm->n && slot >= seg_end_slot) {
                seg++;
                if(seg < tm->n) seg_end_us += tm->t[seg];
                seg_end_slot = us_to_slot(seg_end_us, baud);
            }
            bool mark = seg < tm->n && (seg % 2) == 0;
            in_mark |= mark;
            // a 0 data bit is a light pulse
            if(mark && b >= 1 && b <= 8 && slot % c->cycle_slots == 0) byte &= ~(1u << (b - 1));
        }
        // The forced start-bit pulse of every frame repeats at baud/10, an exact
        // subharmonic of the carrier, so a run of 0xFF still carries a weak
        // carrier that sensitive receivers pick up (a Flipper saw the spaces as
        // marks). With two slots per cycle, odd-slot pulses are in antiphase:
        // 0xE6 lights slots 0,1,4,5 (two even, two odd), cancelling the carrier
        // component. That leaves the spaces ~16 dB below the marks instead of ~9.
        if(!in_mark) byte = ir_space_byte(c);
        if(out && f < out_max) out[f] = byte;
    }
    return frames;
}
