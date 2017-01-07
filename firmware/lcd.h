#ifndef LCD_H
#define LCD_H

/* Turn on the LCD, configuring the ILI9342 and starting the LTDC. */
void lcd_init(void);

/* Framebuf to write into. Will be updated each frame, so don't cache the
 * value. Only write immediately after the semaphore is signalled, within 20ms.
 */
extern uint8_t (*lcd_framebuf)[320];

/* Semaphore that is signalled at the end of each frame, after the display
 * buffers have been rotated. You have about 20ms to draw into the framebuffer.
 */
extern binary_semaphore_t lcd_frame_bs;

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
