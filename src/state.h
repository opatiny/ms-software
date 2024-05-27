#pragma once

#include <stdint.h>

#include <taskGY521.h>
#include "../lib/customUtilities/pidController.h"  // todo: why does #include <pidController.h> not work?

#include "./tasks/taskVl53L1X.h"
#include "pinMapping.h"

typedef int64_t EncoderCounter;

struct EncoderParams {
  int pin1;
  int pin2;
};

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
 *  - previousMode: Previous mode of the motor.
 *  - angleParameter: Serial parameter for the angle of the motor.
 *  - encoderCounts: Number of counts of the encoder since the robot was turned
 *     on.
 *  - currentCommand: Current speed command of the motor.
 */
struct Motor {
  int commandParameter;  // target command
  int modeParameter;
  int angleParameter;
  int previousMode;
  int currentCommand;
  int previousTargetCommand;
  int step; // command step variation per ms
  int pin1;
  int pin2;
  int previousTime;
};

struct WheelsCommands {
  int leftSpeed;
  int rightSpeed;
};

/**
 * The structure for the control of the robot movement. Allows to set the
 * movement mode, speed, etc
 */
struct RobotController {
  int commandParameter;
  int angleParameter;
  int distanceParameter;
  int obstacleDistanceParameter;
  int modeParameter;
  int currentCommand;
  int previousMode;
  int rampStep;
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
  int warningVoltage;
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