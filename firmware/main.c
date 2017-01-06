#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "gfx.h"

#include "ublox.h"
#include "cs2100.h"
#include "measurements.h"
#include "microsd.h"
#include "lcd.h"
#include "speaker.h"


#define BALLCOLOR1		Red
#define BALLCOLOR2		Yellow
#define WALLCOLOR		HTML2COLOR(0x303030)
#define BACKCOLOR		HTML2COLOR(0xC0C0C0)
#define FLOORCOLOR		HTML2COLOR(0x606060)
#define SHADOWALPHA		(255-255*0.2)

float invsqrt(float n);
		float invsqrt(float n) {
			int32_t		i;
			float		x2;

			x2 = n * 0.5F;

			// Convert into an int32 (no binary format conversion)
				i  = *(int32_t *)&n;

			// evil floating point bit level hacking
			i  = 0x5F375A86 - (i >> 1);					// The quake code used 0x5F3759DF but this is better.

			// Convert back to a float (no binary format conversion)
				n  = *(float *)&i;

			n  = n * (1.5F - (x2 * n * n));				// 1st iteration
			//n  = n * (1.5F - (x2 * n * n));			// 2nd iteration for extra precision, this can be removed
			return n;
		}

int main(void)
{
    halInit();
    /*chSysInit();*/

    /* Bring up the ublox, including the 1MHz TIMEPULSE output */
    /*ublox_init(&SD2);*/

    /* Set up the CS2100 to provide the 26MHz GPS-disciplined clock */
    /*cs2100_configure(&I2CD3);*/

    /* Swap the PLL to run off the 26MHz clock */
    /*cs2100_set_pll();*/

    palSetLine(LINE_LED_GRN);

    gfxInit();


	coord_t		width, height, x, y, radius, ballx, bally, dx, floor;
	coord_t		minx, miny, maxx, maxy, winx, winy;
	coord_t		ballcx, ballcy;
	color_t		colour;
	float		ii, spin, dy, spinspeed, h, f, g;
    winx = 1; winy = 1; width = 318; height = 238;
	radius=height/5+height%2+1;	// The ball radius
	ii = 1.0/radius;			// radius as easy math
	floor=height/5-1;			// floor position
	spin=0.0;					// current spin angle on the ball
	spinspeed=0.1;				// current spin speed of the ball
	ballx=width/2;				// ball x position (relative to the ball center)
	bally=height/4;				// ball y position (relative to the ball center)
	dx=.005*width;				// motion in the x axis
	dy=0.0;						// motion in the y axis
	ballcx = 12*radius/5;		// ball x diameter including the shadow
	ballcy = 21*radius/10;		// ball y diameter including the shadow


	minx = miny = 0; maxx = width; maxy = height;		// The clipping window for this frame.


	while(true) {
		// Draw one frame
		gdispStreamStart(winx+minx, winy+miny, maxx-minx, maxy-miny);
		for (y=miny; h = (bally-y)*ii, y<maxy; y++) {
			for (x=minx; x < maxx; x++) {
				g=(ballx-x)*ii;
				f=-.3*g+.954*h;
				if (g*g < 1-h*h) {
					/* The inside of the ball */
					if ((((int)(9-spin+(.954*g+.3*h)*invsqrt(1-f*f))+(int)(2+f*2))&1))
						colour = BALLCOLOR1;
					else
						colour = BALLCOLOR2;
				} else {
					// The background (walls and floor)
					if (y > height-floor) {
						if (x < height-y || height-y > width-x)
							colour = WALLCOLOR;
						else
							colour = FLOORCOLOR;
					} else if (x<floor || x>width-floor)
						colour = WALLCOLOR;
					else
						colour = BACKCOLOR;

					// The ball shadow is darker
					if (g*(g+.4)+h*(h+.1) < 1)
						colour = gdispBlendColor(colour, Black, SHADOWALPHA);
				}
				gdispStreamColor(colour);	/* pixel to the LCD */
			}
		}
		gdispStreamStop();

		// Force a display update if the controller supports it
		/*gdispFlush();*/

		// Calculate the new frame size (note this is a drawing optimisation only)
		minx = ballx - radius; miny = bally - radius;
		maxx = minx + ballcx; maxy = miny + ballcy;
		if (dx > 0) maxx += dx; else minx += dx;
		if (dy > 0) maxy += dy; else miny += dy;
		if (minx < 0) minx = 0;
		if (maxx > width) maxx = width;
		if (miny < 0) miny = 0;
		if (maxy > height) maxy = height;

		// Motion
		spin += spinspeed;
		ballx += dx; bally += dy;
		dx = ballx < radius || ballx > width-radius ? spinspeed=-spinspeed,-dx : dx;
		dy = bally > height-1.75*floor ? -.04*height : dy+.002*height;
    }

    /*coord_t width = 320, height = 240;*/
    /*memset((void*)0x20020000, 0, 320*240*2);*/
    /*gdispCircle(160, 120, 30, Red);*/
    font_t font = gdispOpenFont("DejaVuSans12");
    gdispDrawString(10, 10, "hello world", font, White);
    gdispDrawString(50, 50, "bla bla bla", font, Red);
    gdispFillCircle(160, 120, 30, Green);

    /*gdispFillArea(20, 20, 50, 50, Green);*/
    /*gdispDrawBox(10, 10, width/2, height/2, Yellow);*/
    /*gdispFillArea(width/2, height/2, width/2-10, height/2-10, Blue);*/
    /*gdispDrawLine(5, 30, width-50, height-40, Red);*/

    chThdSleep(TIME_INFINITE);

    /* Turn on the screen and speaker */
    lcd_init();
    speaker_init();

    /* Start up the µSD card, ready for logging */
    /*microsd_init();*/

    /* Wait until GPS lock before starting mains measurements */
    while(!ublox_last_utc.valid) {
        palSetLine(LINE_LED_YLW);
        chThdSleepMilliseconds(500);
        palClearLine(LINE_LED_YLW);
        chBSemWaitTimeout(&ublox_last_utc_bs, MS2ST(500));
    }

    /* Start taking mains measurements */
    /*measurements_init();*/

    while(true) {
        palSetLine(LINE_LED_GRN);
        chThdSleepMilliseconds(500);
        palClearLine(LINE_LED_GRN);
        chThdSleepMilliseconds(500);
    }
}

/* The clock security system calls this if HSE goes away.
 * Best bet is probably to reset everything.
 */
void NMI_Handler(void) {
    NVIC_SystemReset();
}
