#pragma once

#include <stdint.h>

#include <taskGY521.h>
#include "./tasks/taskVl53L1X.h"

typedef int64_t Encoder;

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
  Encoder encoderCounts;
  int currentSpeed;
  int pin1;
  int pin2;
};

struct RobotController {
  int speedParameter;
  int angleParameter;
  int distanceParameter;
  int rampStepParameter;
  int obstacleDistanceParameter;
  int modeParameter;
  int currentSpeed;
  int previousMode;
};

struct Robot {
  Motor leftMotor;
  Motor rightMotor;
  int distances[NB_DISTANCE_SENSORS];
  ImuData imuData;
  RobotController controller;
};

extern Robot robot;