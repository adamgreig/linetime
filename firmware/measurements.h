#ifndef MAINS_H
#define MAINS_H

#include <stdint.h>

void measurements_init(void);

struct mains_cycle {
    /* UTC time of cycle initial zero crossing.
     * Week number, time of week in 2^-32 milliseconds.
     */
    uint16_t utc_week;
    uint64_t utc_tow_sub_ms;

    /* Computed frequency (Hz) and RMS voltage. */
    double frequency, rms;
};

struct mains_waveform {
    /* UTC time of first waveform sample.
     * Week number, time of week in 2^-32 milliseconds.
     */
    uint16_t utc_week;
    uint64_t utc_tow_sub_ms;

    /* Waveform sampled at 5ks/s. */
    int16_t waveform[256];
};

#endif