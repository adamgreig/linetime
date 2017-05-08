#include <stdint.h>
#include <math.h>
#include <complex.h>
#include "mandelbrot.h"
#include "lcd.h"
#include "ch.h"

static int mandelbrot(complex float c, int max_iters) {
    complex float z = 0.0f;
    for(int iters=0; iters<max_iters; iters++) {
        z = z*z + c;
        if(creal(z) >= 2.0 || cimag(z) >= 2.0) {
            return iters;
        }
    }
    return max_iters;
}

void mandelbrot_draw(float xc, float yc, float r, int max_iters)
{
    float xrange = r * 4.0f/3.0f;
    float yrange = r;
    float xmin = xc - xrange/2.0f;
    float ymin = yc - yrange/2.0f;

    for(int x=0; x<320; x++) {
        for(int y=0; y<240; y++) {
            complex float c =      ((((float)x/320.0f)*xrange)+xmin)
                             + I * ((((float)y/240.0f)*yrange)+ymin);
            int iters = mandelbrot(c, max_iters);
            framebuf_back[y][319-x] = (iters * 256) / max_iters;
            chThdYield();
        }
    }
}
