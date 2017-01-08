#ifndef LINETIME_MICROSD_H
#define LINETIME_MICROSD_H

#include <stdint.h>

#define TAG_MEASUREMENT_ZC          ('Z')
#define TAG_MEASUREMENT_WAVE        ('W')
#define TAG_GPS_PVT                 ('P')

void microsd_init(void);
void microsd_log(uint8_t tag, size_t len, void* data);

#endif
