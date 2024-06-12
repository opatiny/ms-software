#pragma once

#include <Arduino.h>

// I2C
extern SemaphoreHandle_t xSemaphoreWire;
#define I2C_SPEED 400000

// debug (U)
enum DebugMode {
  NO_DEBUG,                 // 0
  DEBUG_DISTANCE,           // 1
  DEBUG_IMU,                // 2
  DEBUG_VOLTAGES,           // 3
  DEBUG_BATTERY_LOG_DATA,   // 4
  DEBUG_ENCODERS,           // 5
  DEBUG_BUTTON,             // 6
  DEBUG_MOTORS,             // 7
  DEBUG_RGB_LED,            // 8
  DEBUG_BUZZER,             // 9
  DEBUG_ODOMETRY,           // 10
  DEBUG_SPEED_CALIBRATION,  // 11
  DEBUG_ROBOT_CONTROL,      // 12
  DEBUG_MOTORS_DT,          // 13
  DEBUG_MAIN,               // 14
};

// SERIAL PARAMETERS

#define MAX_PARAM 52
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

// debug
#define PARAM_BATTERY_VOLTAGE 18  // S
#define PARAM_BUZZER 19           // T
#define PARAM_DEBUG 20            // U, 0 = no debug, 1 = debug (serial)

// other parameters
#define PARAM_STATUS \
  21                        // V
                            // W
#define PARAM_CHARGING 23   // X
#define PARAM_WIFI_RSSI 24  // Y
#define PARAM_ERROR 25      // Z

// motors parameters
/**
 * Desired speed for when the motors are moving. This is not the current speed
 * of the motors.
 */
#define PARAM_MOTOR_LEFT_SPEED_CMD 26   // AA
#define PARAM_MOTOR_RIGHT_SPEED_CMD 27  // AB
#define PARAM_MOTOR_LEFT_MODE 28        // AC
#define PARAM_MOTOR_RIGHT_MODE 29       // AD

#define PARAM_BUTTON 32  // AG

#define PARAM_DISTANCE_DEBUG_MODE 33  // AH
#define PARAM_MOTOR_ACC_DURATION 34   // AI

#define PARAM_RGB_LED_COLOR 35       // AJ
#define PARAM_RGB_LED_BRIGHTNESS 36  // AK

// AL

// robot control parameters
/**
 * Command for both wheels of the robot, when the mode is
 * ROBOT_MOVE_SAME_COMMAND.
 */
#define PARAM_ROBOT_COMMAND 38  // AM
/**
 * Target speed for the robot in rpm, used in ROBOT_MOVE mode.
 */
#define PARAM_ROBOT_SPEED 39        // AN
#define PARAM_ROBOT_MODE 40         // AO
#define PARAM_ROBOT_ANGLE_CMD 41    // AP
#define PARAM_OBSTACLE_DISTANCE 42  // AQ

#define PARAM_VCC_VOLTAGE 43  // AR

/**
 * choose RGB LED mode (constant, blink, ...)
 */
#define PARAM_RGB_LED_MODE 44  // AS

#define PARAM_CALIBRATION_MODE 45  // AT
#define PARAM_CALIBRATION_STEP 46  // AU

#define PARAM_CONTROLLER_KP 47  // AV
#define PARAM_CONTROLLER_KI 48  // AW
#define PARAM_CONTROLLER_KD 49  // AX

#define PARAM_STATUS_FLAG_NO_WIFI 0
