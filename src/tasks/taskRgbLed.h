#pragma once

#include <stdint.h>

#define NUMPIXELS 1
#define NB_RGB_COLORS 8

enum RgbLedMode {
  LED_OFF,    // 0
  LED_ON,     // 1
  LED_BLINK,  // 2
};

/**
 * RGB color
 * Each parameter is a value between 0 and 255
 */
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RgbColor;

typedef struct {
  int buttonPressed;
  int batteryEmpty;
  int obstacleDetected;
} RgbLedFlags;

extern RgbLedFlags rgbLedFlags;

const RgbColor colors[NB_RGB_COLORS] = {
    {0, 0, 0},     {255, 0, 0},   {0, 255, 0},   {0, 0, 255},
    {255, 255, 0}, {0, 255, 255}, {255, 0, 255}, {255, 255, 255}};

extern char const colorNames[NB_RGB_COLORS][10];

void taskRgbLed();
