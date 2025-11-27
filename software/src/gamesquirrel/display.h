/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_DISPLAY_H
#define INCLUDED_DISPLAY_H
#include <stdint.h>

typedef enum PixelFormat
{
    PIX_INVALID,
    PIX_RGB888,
    PIX_RGB565,
    PIX_RGB444
} PixelFormat;

typedef enum ImageLayout
{
    IMG_INVALID,
    IMG_PORTRAIT,
    IMG_LANDSCAPE
} ImageLayout;

void DisplayInit(void);
void DisplayFill(void *framebuffer, int x0, int y0, int width, int height);
// FIXME set resolution
// FIXME set bit depth
// FIXME set framerate
// FIXME set orientation
// FIXME copy pixel data
void DisplaySetFormat(PixelFormat pix, ImageLayout layout);

#endif

