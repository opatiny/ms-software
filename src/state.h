#pragma once

#include <stdint.h>

#include <taskGY521.h>

#include "./tasks/taskVl53L1X.h"
#include "pinMapping.h"

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
 *  - speedParameter: Serial parameter for the target speed of the motor.
 *  - modeParameter: Serial parameter for the mode of the motor.
 *  - previousMode: Previous mode of the motor.
 *  - angleParameter: Serial parameter for the angle of the motor.
 *  - encoderCounts: Number of counts of the encoder since the robot was turned
 *     on.
 *  - speed: Current speed of the motor.
 */
struct Motor {
  int speedParameter;
  int modeParameter;
  int previousMode;
  int angleParameter;
  int currentSpeed;
  int pin1;
  int pin2;
};
/**
 * The structure for the control of the robot movement. Allows to set the
 * movement mode, speed, etc
 */
struct RobotController {
  int speedParameter;
  int angleParameter;
  int distanceParameter;
  int obstacleDistanceParameter;
  int modeParameter;
  int currentSpeed;
  int previousMode;
  int rampStep;
};

/**
 * Position of the robot in a cartesian reference frame.
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

struct Odometry {
  Pose pose;
  RobotSpeed speed;
  int time;
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
  ImuData imuData;
  RobotController controller;
  Odometry odometry;
};

extern Robot robot;