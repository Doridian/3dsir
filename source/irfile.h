#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "irenc.h"

typedef struct {
    char name[48];
    bool raw;
    char protocol[16];
    uint32_t address;
    uint32_t command;
    uint32_t frequency;
    uint32_t* data; // raw timings, mark first
    size_t data_n;
} IrSignal;

typedef struct {
    IrSignal* sig;
    size_t n;
} IrFile;

// Parses a Flipper Zero "IR signals file" / "IR library file".
bool irfile_load(const char* path, IrFile* out);
void irfile_free(IrFile* f);

// Turns a signal into a waveform, sent `repeats` times back to back (some
// receivers, e.g. Denon, ignore a single frame). False if unsupported.
bool irfile_signal_timings(const IrSignal* s, unsigned repeats, IrTimings* out);
