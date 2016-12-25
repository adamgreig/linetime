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

/* Stores the most time associated with the last PPS pulse. */
struct ublox_tp_time_t {
    uint32_t tow_ms, tow_sub_ms;
    uint16_t week;
    bool valid;
};
extern struct ublox_tp_time_t ublox_tp_time;

#endif /* UBLOX_H */
