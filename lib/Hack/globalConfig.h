#pragma once

#include <Arduino.h>

#include "debugUtilities.h"

// I2C
extern SemaphoreHandle_t xSemaphoreWire;
#define I2C_SPEED 400000

// SERIAL PARAMETERS

#define MAX_PARAM 78
extern int16_t parameters[MAX_PARAM];

// debug
#define PARAM_DEBUG 0                // A
#define PARAM_DISTANCE_DEBUG_MODE 1  // B

// buzzer parameters
#define PARAM_SOUND 2        // C
#define PARAM_BUZZER_MODE 3  // D

// voltage measurement parameters
#define PARAM_BATTERY_VOLTAGE 4  // E
#define PARAM_VCC_VOLTAGE 5      // F

// RGB LED parameters
/**
 * choose RGB LED mode (constant, blink, ...)
 */
#define PARAM_RGB_LED_MODE 6        // G
#define PARAM_RGB_LED_COLOR 7       // H
#define PARAM_RGB_LED_BRIGHTNESS 8  // I

// push button parameter
#define PARAM_BUTTON 9  // J

// motors parameters
/**
 * Desired speed for when the motors are moving. This is not the current speed
 * of the motors.
 */
#define PARAM_MOTOR_LEFT_COMMAND 10   // K
#define PARAM_MOTOR_RIGHT_COMMAND 11  // L
#define PARAM_MOTOR_LEFT_MODE 12      // M
#define PARAM_MOTOR_RIGHT_MODE 13     // N
#define PARAM_MOTOR_ACC_DURATION 14   // O

// robot navigation parameters
/**
 * Command for both wheels of the robot, when the mode is
 * ROBOT_MOVE_SAME_COMMAND.
 */
#define PARAM_ROBOT_COMMAND 15  // P
/**
 * Target speed for the robot in rpm, used in ROBOT_MOVE mode.
 */
#define PARAM_ROBOT_WHEELS_SPEED 16  // Q, target wheel speed in rpm
#define PARAM_ROBOT_SPEED_LIN 17     // R, target linear speed in mm/s
#define PARAM_ROBOT_SPEED_ANG 18     // S, target angular speed in deg/s
#define PARAM_ROBOT_MODE 19          // T
#define PARAM_ROBOT_ANGLE_CMD 20     // U
#define PARAM_OBSTACLE_DISTANCE 21   // V

// speed calibration parameters
#define PARAM_CALIBRATION_MODE 22  // W
#define PARAM_CALIBRATION_STEP 23  // X

#define PARAM_ODOMETRY_RESET 24  // Y

// IMU parameteers
// accelerations are in m/s^2 * 100
#define PARAM_ACCELERATION_X 26  // AA
#define PARAM_ACCELERATION_Y 27  // AB
#define PARAM_ACCELERATION_Z 28  // AC
// rotations are in rad/s * 100
#define PARAM_ROTATION_X 29  // AD
#define PARAM_ROTATION_Y 30  // AE
#define PARAM_ROTATION_Z 31  // AF

// distance sensors parameters (distances are all in mm)
#define PARAM_DISTANCE_LEFT 32         // AG
#define PARAM_DISTANCE_FRONT_LEFT 33   // AH
#define PARAM_DISTANCE_FRONT 34        // AI
#define PARAM_DISTANCE_FRONT_RIGHT 35  // AJ
#define PARAM_DISTANCE_RIGHT 36        // AK

// controllers parameters

// pick which controller to use (set on or off using boolean value)
#define PARAM_LINEAR_CONTROLLER 52   // BA
#define PARAM_ANGULAR_CONTROLLER 53  // BB
#define PARAM_WALLS_CONTROLLER 54    // BC

// todo: verify the numbers
// wheels speed controller
#define PARAM_WHEEL_KP 55  // BD
#define PARAM_WHEEL_KI 56  // BE
#define PARAM_WHEEL_KD 57  // BF

// robot linear speed controller
#define PARAM_LINEAR_KP 58  // BG
#define PARAM_LINEAR_KI 59  // BH
#define PARAM_LINEAR_KD 60  // BI

// robot angular speed controller
#define PARAM_ANGULAR_KP 61  // BJ
#define PARAM_ANGULAR_KI 62  // BK
#define PARAM_ANGULAR_KD 63  // BL

// various parameters
#define PARAM_STATUS 75     // BX
#define PARAM_WIFI_RSSI 76  // BY
#define PARAM_ERROR 77      // BZ
#define PARAM_STATUS_FLAG_NO_WIFI 0