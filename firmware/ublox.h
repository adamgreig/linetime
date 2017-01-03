/*
 * uBlox GPS receiver
 * 2014, 2016 Adam Greig
 */

#ifndef UBLOX_H
#define UBLOX_H

#include "ch.h"
#include "hal.h"

/* Call to start ublox processing thread on specified Serial port */
void ublox_init(SerialDriver* seriald);

/* Stores the time associated with the previous PPS pulse. */
struct ublox_utc {
    uint8_t year, month, day, hour, minute, second;
    bool valid;
};

/* Signals when a new UTC reading is received, and its value. */
extern binary_semaphore_t ublox_last_utc_bs;
extern struct ublox_utc ublox_last_utc;

#endif /* UBLOX_H */
