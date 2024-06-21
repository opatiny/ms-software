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
  int lowSpeedNbCounts;
  EncoderCounter lowSpeedCounts;
  double lowSpeed;  // computed with X/delta t, more precise at low speeds
  uint32_t previousTime;
};

/**
 * Structure containing the wheel speed computed through different methods.
 * The choice of the actual speed is based on the amplitude of the speed.
 */
struct WheelSpeeds {
  double lowSpeed;   // computed with X/delta t, more precise at low speeds
  double highSpeed;  // computed with delta X/T, more precise at high speeds
};
/**
 * Motor structure.
 *  - commandParameter: Serial parameter for the target speed of the motor.
 *  - modeParameter: Serial parameter for the mode of the motor.
 *  - angleParameter: Serial parameter for the angle of the motor.
 *  - accDurationParameter: Serial parameter defining the duration of
 *    accelerations.
 *  - previousMode: Previous mode of the motor.
 *  - currentCommand: Current speed command applied to the motor.
 *  - previousTargetCommand: Previous target command of the motor.
 *  - step: Variation of the command required per ms for when motor is
 *    accelerating. Example: 1 step/ms.
 *  - pin1: Pin 1 of the motor.
 *  - pin2: Pin 2 of the motor.
 *  - previousTime: Time of the previous update of the motor in ms.
 *  - regressions: Structure containing the coefficients of the polynomial
 *    regressions to convert desired motor speed in rpm to command.
 *  - wheelSpeed: Current speed of the wheel in rpm.
 */
struct Motor {
  int commandParameter;  // target command
  int modeParameter;
  int angleParameter;
  int accDurationParameter;
  int previousMode;
  int currentCommand;
  int previousTargetCommand;
  int step;
  int pin1;
  int pin2;
  uint32_t previousTime;  // time in ms!!
  Regressions regressions;
  double wheelSpeed;
  WheelSpeeds wheelSpeeds;
};

/**
 * Control both wheels speeds with a separate PID controller.
 */
struct WheelsSpeedController {
  PidController left;
  PidController right;
  bool clearControllers;
};

/**
 * Control the robot's angular and linear speed with a PID controller.
 */
struct RobotSpeedController {
  PidController linear;
  PidController angular;
  bool clearControllers;
};

/**
 * The structure for the control of the robot movement. Allows to set the
 * movement mode, speed, etc
 */
struct RobotNavigation {
  int commandParameter;
  int speedParameter;
  int angleParameter;
  int distanceParameter;
  int obstacleDistanceParameter;
  int modeParameter;
  int currentCommand;
  int currentSpeed;
  int previousMode;
  WheelsSpeedController wheelsSpeedController;
  RobotSpeedController robotSpeedController;
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
 * - time: The time at the last update of the odometry in microseconds.
 */
struct Odometry {
  Pose pose;
  RobotSpeed speed;
  uint32_t time;  // time in us -> high precision required
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
  RobotNavigation navigation;
  Odometry odometry;
  VoltageMeasurement battery;
  VoltageMeasurement vcc;
};

extern Robot robot;