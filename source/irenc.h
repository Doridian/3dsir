#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// A waveform as alternating durations in microseconds: t[0] is a mark
// (carrier on), t[1] a space (carrier off), t[2] a mark, and so on.
typedef struct {
    uint32_t* t;
    size_t n;
    size_t cap;
    uint32_t freq; // desired carrier frequency in Hz
} IrTimings;

// The UART bit rates ir:u can do that are usable as IR carriers.
typedef struct {
    uint32_t baud;
    uint8_t iru_value; // value passed to IRU_SetBitRate
    uint8_t cycle_slots; // UART bits per carrier cycle (carrier = baud / cycle_slots)
} IrCarrier;

typedef enum {
    // One pulse per bit: 0x00 bytes at 36000/38400 baud. Exact frequency, but
    // every tenth pulse is missing (the stop bit).
    IrCarrierDense,
    // One pulse every other bit: 0x55 bytes at 72000/96000/115200 baud. The
    // start and stop bits fall in phase, so the carrier is gapless, but only
    // 36/48/57.6 kHz are available and the duty cycle is ~9%.
    IrCarrierClean,
} IrCarrierStyle;

void ir_timings_init(IrTimings* tm);
void ir_timings_free(IrTimings* tm);
void ir_timings_clear(IrTimings* tm);
void ir_timings_mark(IrTimings* tm, uint32_t us);
void ir_timings_space(IrTimings* tm, uint32_t us);

// Repeats a waveform (which must end in a mark) `times` times in total,
// separated by gap_us of silence.
void ir_timings_repeat(IrTimings* tm, unsigned times, uint32_t gap_us);

// Encodes one button press of a Flipper-style "parsed" signal (including the
// protocol's minimum repeat count). Returns false for unknown protocols.
bool ir_encode_parsed(const char* protocol, uint32_t address, uint32_t command, IrTimings* out);

// Picks the ir:u bit rate closest to the requested carrier frequency.
const IrCarrier* ir_pick_carrier(uint32_t freq, IrCarrierStyle style);

// Renders a waveform into raw UART bytes for the SC16IS750 in IrDA SIR mode.
//
// In IrDA mode every 0 bit on the wire becomes a 3/16-bit-wide light pulse and
// every 1 bit becomes darkness. At 38400 baud a 0x00 byte is therefore nine
// pulses at 38.4 kHz (start bit + 8 data bits) followed by one missing pulse
// (stop bit): a ~19% duty-cycle 38 kHz carrier. A 0xFF byte is a lone start-bit
// pulse, which a TV's band-pass receiver ignores, so it serves as a space.
//
// With cycle_slots == 2 only even bit slots are lit, giving 0x55 in marks.
//
// Returns the number of bytes needed; writes at most out_max bytes to out
// (out may be NULL to just measure).
size_t ir_render_uart(const IrTimings* tm, const IrCarrier* c, uint8_t* out, size_t out_max);

// The byte ir_render_uart emits for a UART frame that lies entirely in a space.
uint8_t ir_space_byte(const IrCarrier* c);
