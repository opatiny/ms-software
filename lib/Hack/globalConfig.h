#pragma once

#include <Arduino.h>

// I2C
extern SemaphoreHandle_t xSemaphoreWire;
#define I2C_SPEED 400000

// debug
enum DebugMode { NO_DEBUG, DEBUG_DISTANCE, DEBUG_IMU, DEBUG_BATTERY };

#define DEBUG_BOOT_MODE DEBUG_BATTERY

// default parameters values

// SERIAL PARAMETERS
#define SERIAL_SPEED 115200

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

// distance sensors parameters (distances are all in mm)
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

// motors parameters
#define PARAM_MOTOR_LEFT_SPEED 16   // Q
#define PARAM_MOTOR_RIGHT_SPEED 17  // R

// debug
#define PARAM_BATTERY_VOLTAGE 18  // S
#define PARAM_BUZZER 19           // T
#define PARAM_DEBUG 20            // U, 0 = no debug, 1 = debug (serial)

// other parameters
#define PARAM_STATUS 21     // V
#define PARAM_BATTERY 22    // W
#define PARAM_CHARGING 23   // X
#define PARAM_WIFI_RSSI 24  // Y
#define PARAM_ERROR 25      // Z

#define PARAM_STATUS_FLAG_NO_WIFI 0
