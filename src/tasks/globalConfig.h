#pragma once

#include <Arduino.h>

#define MAX_PARAM 26
extern int16_t parameters[MAX_PARAM];

// example
#define PARAM_TEMPERATURE 0  // A
#define PARAM_HUMIDITY 1     // B
#define PARAM_PRESSURE 2     // C ...

#define PARAM_STATUS 21     // V
#define PARAM_BATTERY 22    // W
#define PARAM_CHARGING 23   // X
#define PARAM_WIFI_RSSI 24  // Y
#define PARAM_ERROR 25      // Z

#define PARAM_STATUS_FLAG_NO_WIFI 0
