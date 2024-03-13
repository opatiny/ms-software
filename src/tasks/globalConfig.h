#pragma once

#include <Arduino.h>

// I2C
extern SemaphoreHandle_t xSemaphoreWire;
#define IMU_ADDRESS 0x68
// distance sensors
#define VL53_FRONT_ADRESS 0x29

// serial parameters
#define MAX_PARAM 26
extern int16_t parameters[MAX_PARAM];

// IMU parameteers
// accelerations are in g -> 1g = 9.81 m/s^2
#define PARAM_ACCELERATION_X 0  // A
#define PARAM_ACCELERATION_Y 1  // B
#define PARAM_ACCELERATION_Z 2  // C
#define PARAM_ROTATION_X 3      // D
#define PARAM_ROTATION_Y 4      // E
#define PARAM_ROTATION_Z 5      // F

// distance sensors
// distances are all in mm
#define PARAM_DISTANCE_FRONT 6  // G

#define PARAM_STATUS 21     // V
#define PARAM_BATTERY 22    // W
#define PARAM_CHARGING 23   // X
#define PARAM_WIFI_RSSI 24  // Y
#define PARAM_ERROR 25      // Z

#define PARAM_STATUS_FLAG_NO_WIFI 0
