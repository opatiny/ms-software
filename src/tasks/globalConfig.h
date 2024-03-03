#pragma once

#include <Arduino.h>

#define MAX_PARAM 26
extern int16_t parameters[MAX_PARAM];

// example
#define PARAM_TEMPERATURE 0  // A
#define PARAM_HUMIDITY 1     // B
#define PARAM_PRESSURE 2     // C ...

#define PARAM_ERROR 25      // Z
