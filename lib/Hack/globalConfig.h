#pragma once

#include <Arduino.h>

// I2C
extern SemaphoreHandle_t xSemaphoreWire;
#define IMU_ADDRESS 0x68
// distance sensors
#define NB_DISTANCE_SENSORS 2

#define VL53_LEFT_ADDRESS 0x29
#define VL53_FRONT_LEFT_ADDRESS 0x30
#define VL53_FRONT_ADDRESS 0x31
#define VL53_FRONT_RIGHT_ADDRESS 0x32
#define VL53_RIGHT_ADDRESS 0x33

// SERIAL PARAMETERS
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
#define PARAM_DISTANCE_LEFT 6         // G
#define PARAM_DISTANCE_FRONT_LEFT 7   // H
#define PARAM_DISTANCE_FRONT 8        // I
#define PARAM_DISTANCE_FRONT_RIGHT 9  // J
#define PARAM_DISTANCE_RIGHT 10       // K

#define PARAM_OFFSET_FRONT 11        // L
#define PARAM_OFFSET_FRONT_LEFT 12   // M
#define PARAM_OFFSET_LEFT 13         // N
#define PARAM_OFFSET_RIGHT 14        // O
#define PARAM_OFFSET_FRONT_RIGHT 15  // P

// other parameters
#define PARAM_DEBUG 20      // U, 0 = no debug, 1 = debug
#define PARAM_STATUS 21     // V
#define PARAM_BATTERY 22    // W
#define PARAM_CHARGING 23   // X
#define PARAM_WIFI_RSSI 24  // Y
#define PARAM_ERROR 25      // Z

#define PARAM_STATUS_FLAG_NO_WIFI 0
