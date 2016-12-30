#ifndef MICROSD_H
#define MICROSD_H

#include <stdint.h>

#define TAG_MEASUREMENT_ZC          ('Z')
#define TAG_MEASUREMENT_WAVE        ('W')
#define TAG_GPS_PVT                 ('P')
#define TAG_GPS_TP                  ('T')

void microsd_init(void);
void microsd_log(uint8_t tag, size_t len, void* data);

#endif
