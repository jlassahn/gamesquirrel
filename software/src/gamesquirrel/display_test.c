
#include "gamesquirrel/display.h"
#include "gamesquirrel/core.h"
#include <stdio.h>
#include <math.h>

uint32_t ADCRead(int n); // FIXME fake

typedef struct PixRGB
{
    int r;
    int g;
    int b;
} PixRGB;

typedef struct Sphere
{
    int x;
    int y;
    int r;
} Sphere;

static int frame_count = 0;
static Sphere sphere =
{
    160,
    120,
    60
};
static float light_x = 0.707f;
static float light_y = 0.707f;
static float light_z = 0.0f;

static void DoPixel(PixRGB *pix, int x, int y)
{
    const PixRGB background = {100, 80, 200};

    float dx = x - sphere.x;
    float dy = y - sphere.y;
    float z2 = (sphere.r*sphere.r - dx*dx - dy*dy);
    if (z2 < 0.0f)
    {
        *pix = background;
        return;
    }
    float dz = sqrtf(z2);

    float light =
        light_x*dx/sphere.r +
        light_y*dy/sphere.r +
        light_z*dz/sphere.r;

    if (light < 0.0f)
        light = 0.0f;
    light = (light *0.5f) + 0.5;

    PixRGB pixel;
    pixel.r = (int)(255.0f*light);
    pixel.g = 0;
    pixel.b = 0;
    *pix = pixel;
}

void DisplayTest(void)
{
    static uint16_t framebuffer[320*240];

    uint32_t t0 = TimeMicroseconds();

    sphere.x = 160 + (((int32_t)ADCRead(0) - 0x8000) >> 8);
    sphere.y = 120 - (((int32_t)ADCRead(1) - 0x8000) >> 8);

    light_x = cosf(frame_count*(3.14159f/50.0f));
    light_z = sinf(frame_count*(3.14159f/50.0f));
    light_y = 0.0f;

    uint16_t *p = framebuffer;
    for (int y=0; y<240; y++)
    for (int x=0; x<320; x++)
    {
        PixRGB pixel;
        DoPixel(&pixel, x, y);
        uint16_t n =
            (pixel.r & 0x00F8) |
            ((pixel.g >> 5) & 0x0003) |
            ((pixel.g << 11) & 0xE000) |
            ((pixel.b << 5) & 0x1F00);
        *p = n;
        p ++;
    }

    uint32_t t1 = TimeMicroseconds();

    DisplayFill(framebuffer, 0, 0, 320, 240);

    uint32_t t2 = TimeMicroseconds();

    printf("draw: %lu fill: %lu\r\n", t1-t0, t2 - t1);
    frame_count = (frame_count+1)%100;
}

