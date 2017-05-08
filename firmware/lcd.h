#ifndef LINETIME_LCD_H
#define LINETIME_LCD_H

#include "ch.h"

/* Turn on the LCD, configuring the ILI9342 and starting the LTDC. */
void lcd_init(void);

/* Framebuf to write into. Will be updated each frame, so don't cache the
 * value. Only write immediately after the event is signalled, within 15ms.
 */
extern uint8_t (*lcd_framebuf)[320];
extern uint8_t (*framebuf_front)[320];
extern uint8_t (*framebuf_back)[320];

/* Event that is broadcast at the end of each frame, after the display
 * buffers have been rotated. You have about 15ms to draw into the framebuffer.
 */
extern event_source_t lcd_frame_evt;

/* Colours in the lookup table */
#define LCD_BLACK   (0)
#define LCD_WHITE   (1)
#define LCD_RED     (2)
#define LCD_GREEN   (3)
#define LCD_BLUE    (4)
#define LCD_YELLOW  (5)
#define LCD_PURPLE  (6)
#define LCD_CYAN    (7)

#endif
