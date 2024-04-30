#pragma once

#include <stdint.h>

#define NUMPIXELS 1

/**
 * RGB color
 * Each parameter is a value between 0 and 255
 */
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RgbColor;

const RgbColor colors[] = {{255, 0, 0},     {0, 255, 0},   {0, 0, 255},
                           {255, 255, 0},   {0, 255, 255}, {255, 0, 255},
                           {255, 255, 255}, {0, 0, 0}};

void taskRgbLed();
