#pragma once

/**
 * Specify the information you want to print to serial.
 * - RANGING: Normal mode, display distance of all sensors every time a
 * measurement is done.
 * - CONSTANT: Log the time and each distances every time a measurement is done
 * as csv data. Typically used to see variability when distance is constant.
 * - CALIBRATION: Only print the distances when push button is pressed, csv
 * format as well.
 */
enum DistanceDebugMode { RANGING, CONSTANT, CALIBRATION };

// I2C addresses
#define VL53_LEFT_ADDRESS 0x2A
#define VL53_FRONT_LEFT_ADDRESS 0x2B
#define VL53_FRONT_ADDRESS 0x2C
#define VL53_FRONT_RIGHT_ADDRESS 0x2D
#define VL53_RIGHT_ADDRESS 0x2E

/**
 * Time to perform a measurement. 140 ms is the min time for max range of 4
 * meters.
 */
#define TIMING_BUDGET 140          // ms
#define VL53_DEFAULT_ADDRESS 0x29  // 41

// prototypes
void taskVL53L1X();