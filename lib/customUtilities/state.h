#pragma once

#include <stdint.h>

#include "../Hack/taskGY521.h"
#include "pidController.h"
#include "pinMapping.h"
#include "regressions.h"

typedef int64_t EncoderCounter;

/**
 * Structure containing all the encoder data.
 * - counts: Number of counts of the encoder since the robot was turned on.
 * - previousCounts: Number of counts of the encoder at the previous odometry
 *   update.
 * - pin1: Pin 1 of the encoder.
 * - pin2: Pin 2 of the encoder.
 */
struct Encoder {
  EncoderCounter counts;
  EncoderCounter previousCounts;
  int pin1;
  int pin2;
};

/**
 * Motor structure.
 *  - commandParameter: Serial parameter for the target speed of the motor.
 *  - modeParameter: Serial parameter for the mode of the motor.
 *  - angleParameter: Serial parameter for the angle of the motor.
 *  - accDurationParameter: Serial parameter defining the duration of
 * accelerations.
 *  - previousMode: Previous mode of the motor.
 *  - currentCommand: Current speed command of the motor.
 * - previousTargetCommand: Previous target command of the motor.
 * - step: Variation of the command required per ms for when motor is
 * accelerating. Example: 1 step/ms.
 * - pin1: Pin 1 of the motor.
 * - pin2: Pin 2 of the motor.
 * - previousTime: Time of the previous update of the motor.
 * - regressions: Structure containing the coefficients of the polynomial
 * regressions to convert desired motor speed in rpm to command.
 */
struct Motor {
  int commandParameter;  // target command
  int modeParameter;
  int angleParameter;
  int accDurationParameter;
  int previousMode;
  int currentCommand;
  int previousTargetCommand;
  int step;  // command step variation per ms
  int pin1;
  int pin2;
  int previousTime;
  Regressions regressions;
};

struct WheelsCommands {
  int leftCommand;
  int rightCommand;
};

/**
 * The structure for the control of the robot movement. Allows to set the
 * movement mode, speed, etc
 */
struct RobotController {
  int commandParameter;
  int speedParameter;
  int angleParameter;
  int distanceParameter;
  int obstacleDistanceParameter;
  int modeParameter;
  int currentCommand;
  int currentSpeed;
  int previousMode;
  PidController angularPid;
  PidController linearPid;
  WheelsCommands wheelsCommands;
};

/**
 * Position of the robot in a cartesian reference frame in meters and radians.
 */
struct Pose {
  double x;
  double y;
  double theta;
};

struct RobotSpeed {
  double v;
  double omega;
};

/**
 * Structure for the odometry data.
 * - pose: The pose of the robot in meters and radians.
 * - speed: The speed of the robot in m/s and rad/s.
 * - time: The time at the last update of the odometry in milliseconds.
 * - leftWheelSpeed: The speed of the left wheel in rpm.
 * - rightWheelSpeed: The speed of the right wheel in rpm.
 */
struct Odometry {
  Pose pose;
  RobotSpeed speed;
  int time;
  float leftWheelSpeed;
  float rightWheelSpeed;
};

struct VoltageMeasurement {
  int voltageParameter;
  int pin;
  double warningVoltage;
};

/**
 * The highest level structure for the robot. Contains all the robot's data.
 */
struct Robot {
  Motor leftMotor;
  Motor rightMotor;
  Encoder leftEncoder;
  Encoder rightEncoder;
  int distances[NB_DISTANCE_SENSORS];
  // bool walls[NB_DISTANCE_SENSORS];
  ImuData imuData;
  RobotController controller;
  Odometry odometry;
  VoltageMeasurement battery;
  VoltageMeasurement vcc;
};

extern Robot robot;