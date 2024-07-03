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
#define PARAM_DEBUG 20                // U
#define PARAM_DISTANCE_DEBUG_MODE 33  // AH

// buzzer parameters
#define PARAM_SOUND 16        // Q
#define PARAM_BUZZER_MODE 18  // S

// voltage measurement parameters
#define PARAM_BATTERY_VOLTAGE 19  // T
#define PARAM_VCC_VOLTAGE 43      // AR

// RGB LED parameters
/**
 * choose RGB LED mode (constant, blink, ...)
 */
#define PARAM_RGB_LED_MODE 44        // AS
#define PARAM_RGB_LED_COLOR 35       // AJ
#define PARAM_RGB_LED_BRIGHTNESS 43  // AK

// motors parameters
/**
 * Desired speed for when the motors are moving. This is not the current speed
 * of the motors.
 */
#define PARAM_MOTOR_LEFT_COMMAND 26   // AA
#define PARAM_MOTOR_RIGHT_COMMAND 27  // AB
#define PARAM_MOTOR_LEFT_MODE 28      // AC
#define PARAM_MOTOR_RIGHT_MODE 29     // AD
#define PARAM_MOTOR_ACC_DURATION 34   // AI

// robot navigation parameters
/**
 * Command for both wheels of the robot, when the mode is
 * ROBOT_MOVE_SAME_COMMAND.
 */
#define PARAM_ROBOT_COMMAND 38  // AM
/**
 * Target speed for the robot in rpm, used in ROBOT_MOVE mode.
 */
#define PARAM_ROBOT_WHEELS_SPEED 39  // AN, target wheel speed in rpm
#define PARAM_ROBOT_SPEED_LIN 50     // AY, target linear speed in mm/s
#define PARAM_ROBOT_SPEED_ANG 37     // AL, target angular speed in deg/s
#define PARAM_ROBOT_MODE 40          // AO
#define PARAM_ROBOT_ANGLE_CMD 41     // AP
#define PARAM_OBSTACLE_DISTANCE 42   // AQ

// speed calibration parameters
#define PARAM_CALIBRATION_MODE 45  // AT
#define PARAM_CALIBRATION_STEP 46  // AU

// controllers parameters: pick which controller to use (boolean value)
#define PARAM_LINEAR_CONTROLLER 30   // AE
#define PARAM_ANGULAR_CONTROLLER 31  // AF
#define PARAM_WALLS_CONTROLLER 47    // AV

// IMU parameteers
// accelerations are in m/s^2 * 100
#define PARAM_ACCELERATION_X 0  // A
#define PARAM_ACCELERATION_Y 1  // B
#define PARAM_ACCELERATION_Z 2  // C
// rotations are in rad/s * 100
#define PARAM_ROTATION_X 3  // D
#define PARAM_ROTATION_Y 4  // E
#define PARAM_ROTATION_Z 5  // F

// distance sensors parameters (distances are all in mm)
#define PARAM_DISTANCE_LEFT 6         // G
#define PARAM_DISTANCE_FRONT_LEFT 7   // H
#define PARAM_DISTANCE_FRONT 8        // I
#define PARAM_DISTANCE_FRONT_RIGHT 9  // J
#define PARAM_DISTANCE_RIGHT 10       // K

// push button parameter
#define PARAM_BUTTON 32  // AG

// controllers parameters

// todo: verify the numbers
// wheels speed controller
#define PARAM_WHEEL_KP 52  // BA
#define PARAM_WHEEL_KI 53  // BB
#define PARAM_WHEEL_KD 54  // BC

// robot linear speed controller
#define PARAM_LINEAR_KP 55  // BD
#define PARAM_LINEAR_KI 56  // BE
#define PARAM_LINEAR_KD 57  // BF

// robot angular speed controller
#define PARAM_ANGULAR_KP 58  // BG
#define PARAM_ANGULAR_KI 59  // BH
#define PARAM_ANGULAR_KD 60  // BI

// various parameters
// V
// W
#define PARAM_STATUS 75     // BX
#define PARAM_WIFI_RSSI 76  // BY
#define PARAM_ERROR 77      // BZ
#define PARAM_STATUS_FLAG_NO_WIFI 0