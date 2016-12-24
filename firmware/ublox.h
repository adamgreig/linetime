/*
 * uBlox GPS receiver
 * 2014, 2016 Adam Greig
 */

#ifndef UBLOX_H
#define UBLOX_H

#include <stdint.h>
#include "hal.h"

/* Call to start ublox processing thread on specified Serial port */
void ublox_init(SerialDriver* seriald);

#endif /* UBLOX_H */
