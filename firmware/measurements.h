#ifndef MAINS_H
#define MAINS_H

#include <stdint.h>

void measurements_init(void);

struct mains_cycle {
    /* UTC time of PPS immediately before cycle initial zero crossing.
     * utc_year is years-since-2000.
     */
    uint8_t utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_second;

    /* Nanoseconds since last PPS. */
    uint32_t nanoseconds;

    /* Measured period, in nanoseconds. */
    uint32_t period_ns;

    /* Measured RMS voltage, in 2^-15 volts (UQ9.7). */
    uint16_t rms;
} __attribute__((packed));

struct mains_waveform {
    union {
        /* UTC time of PPS pulse immediately before first sample.
         * utc_year is years-since-2000.
         */
        struct {
        uint8_t utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_second;
        } __attribute__((packed));

        /* Internal use only. Sorry. Pretend this isn't here. */
        uint32_t _pps_timestamp;
    };

    /* Nanoseconds between last PPS and first sample. */
    uint32_t nanoseconds;

    /* Waveform sampled at 1ks/s. */
    int8_t waveform[128];
} __attribute__((packed));

#endif
