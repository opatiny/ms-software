#pragma once

#define NB_DISTANCE_SENSORS 2

enum DistanceDebugMode { RANGING, CONSTANT, CALIBRATION };

// I2C addresses
#define VL53_LEFT_ADDRESS 0x2A
#define VL53_FRONT_LEFT_ADDRESS 0x2B
#define VL53_FRONT_ADDRESS 0x2C
#define VL53_FRONT_RIGHT_ADDRESS 0x2D
#define VL53_RIGHT_ADDRESS 0x2E

// shutdown pins: device is off when pin is low
#define XSHUT_PIN_LEFT D8
#define XSHUT_PIN_FRONT_LEFT D7
#define XSHUT_PIN_FRONT 0
#define XSHUT_PIN_FRONT_RIGHT 0
#define XSHUT_PIN_RIGHT 0

#define TIMING_BUDGET 140  // ms
#define VL53_DEFAULT_ADDRESS 0x29

// global variables
extern bool distance_calibration_button_pressed;

// prototypes
void taskVL53L1X();