#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "lcd.h"
#include "ublox.h"
#include "gui.h"


/**************************************************************** PROTOTYPES */
static void draw_circle(uint16_t cx, uint16_t cy, uint16_t r, uint8_t colour);
static void draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                      uint8_t colour);
static void draw_clock(uint16_t cx, uint16_t cy, uint16_t r);
/*****************************************************************************/

/********************************************************* DRAWING FUNCTIONS */
static void set_pixel(uint16_t x, uint16_t y, uint8_t colour)
{
    /* Compensate for stupid screen horizontal flipping */
    lcd_framebuf[y][319-x] = colour;
}

static void draw_circle(uint16_t cx, uint16_t cy, uint16_t r, uint8_t colour)
{
    int16_t x = 0;
    int16_t y = r;
    int16_t p = (5 - r*4)/4;
    do {
        set_pixel(cx+x, cy+y, colour);
        set_pixel(cx+x, cy-y, colour);
        set_pixel(cx-x, cy+y, colour);
        set_pixel(cx-x, cy-y, colour);
        set_pixel(cx+y, cy+x, colour);
        set_pixel(cx+y, cy-x, colour);
        set_pixel(cx-y, cy+x, colour);
        set_pixel(cx-y, cy-x, colour);
        x++;
        if(p<0) {
            p += 2*x + 1;
        } else {
            y--;
            p += 2*(x-y)+1;
        }
    } while(x < y);
}

static void draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
               uint8_t colour)
{
    uint16_t dy = 2 * abs(y1 - y0);
    uint16_t dx = 2 * abs(x1 - x0);
    int16_t sx = x0 > x1 ? -1 : 1;
    int16_t sy = y0 > y1 ? -1 : 1;
    set_pixel(x0, y0, colour);
    if(dx > dy) {
        int16_t f = dy - (dx/2);
        for(; x0 != x1; x0 += sx) {
            if(f >= 0) {
                y0 += sy;
                f -= dx;
            }
            f += dy;
            set_pixel(x0, y0, colour);
        }
    } else {
        int16_t f = dx - (dy/2);
        for(; y0 != y1; y0 += sy) {
            if(f >= 0) {
                x0 += sx;
                f -= dy;
            }
            f += dx;
            set_pixel(x0, y0, colour);
        }
    }
}
/*****************************************************************************/

/*********************************************************** CLOCK FUNCTIONS */
static void draw_clock(uint16_t cx, uint16_t cy, uint16_t r)
{
    /* Clock outline */
    draw_circle(cx, cy, r, LCD_WHITE);

    /* Draw ticks on the hours */
    for(float theta = 0.0f; theta < M_PI/2.0f; theta += M_PI/6.0f) {
        uint16_t len = theta == 0.0f ? 20 : 10;
        uint16_t dx0 = (uint16_t)((r-len)*sinf(theta));
        uint16_t dx1 = (uint16_t)((r    )*sinf(theta));
        uint16_t dy0 = (uint16_t)((r-len)*cosf(theta));
        uint16_t dy1 = (uint16_t)((r    )*cosf(theta));
        draw_line(cx+dx0, cy+dy0, cx+dx1, cy+dy1, LCD_WHITE);
        draw_line(cx-dx0, cy-dy0, cx-dx1, cy-dy1, LCD_WHITE);
        draw_line(cx+dy0, cy-dx0, cx+dy1, cy-dx1, LCD_WHITE);
        draw_line(cx-dy0, cy+dx0, cx-dy1, cy+dx1, LCD_WHITE);
    }

    /* Compute angles for clock hands */
    float theta_h = ((float)(ublox_last_utc.hour%12)/12.0) * 2.0f * M_PI;
    float theta_m = ((float) ublox_last_utc.minute  /60.0) * 2.0f * M_PI;
    float theta_s = ((float) ublox_last_utc.second  /60.0) * 2.0f * M_PI;
    theta_h += theta_m / 12.0f;

    draw_line(cx, cy,
              cx+(r/2)*sinf(theta_h),
              cy-(r/2)*cosf(theta_h),
              LCD_RED);
    draw_line(cx, cy,
              cx+((2*r)/3)*sinf(theta_m),
              cy-((2*r)/3)*cosf(theta_m),
              LCD_GREEN);
    draw_line(cx, cy,
              cx+((4*r)/5)*sinf(theta_s),
              cy-((4*r)/5)*cosf(theta_s),
              LCD_BLUE);
}
/*****************************************************************************/

/******************************************************************* THREADS */
static THD_WORKING_AREA(clock_thd_wa, 512);
static THD_FUNCTION(clock_thd, arg)
{
    (void)arg;
    while(true) {
        chBSemWait(&lcd_frame_bs);
        memset(lcd_framebuf, 0, 320*240);
        draw_clock(320/2, 240/2, 100);

        /* Slightly weird line-boldening technique.
         * Note we can't set any pixel ahead of where we're looking
         * or we'll keep duplicating it forever in that direction.
         *    - - - >
         *  | x x            x = OK to set
         *  | x X            X = current pixel
         *  | x
         *  v
         */
        for(size_t x=1; x<319; x++) {
            for(size_t y=1; y<239; y++) {
                uint8_t p = lcd_framebuf[y][x];
                if(p != 0) {
                    lcd_framebuf[y-1][x-1] = p;
                    lcd_framebuf[y-1][x  ] = p;
                    lcd_framebuf[y  ][x-1] = p;
                    lcd_framebuf[y+1][x-1] = p;
                }
            }
        }
    }
}
/*****************************************************************************/

/********************************************************** PUBLIC FUNCTIONS */
void gui_init()
{
    chThdCreateStatic(clock_thd_wa, sizeof(clock_thd_wa), NORMALPRIO-1,
                      clock_thd, NULL);
}
/*****************************************************************************/
