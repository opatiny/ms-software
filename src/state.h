#pragma once

#include <stdint.h>

#include <taskGY521.h>
#include "./tasks/taskVl53L1X.h"

typedef int64_t EncoderCounter;

struct Encoder {
  EncoderCounter counter;
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

struct Pose {
  double x;
  double y;
  double theta;
};

struct Odometry {
  Pose previousPose;
  int previousTime;
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
};

extern Robot robot;