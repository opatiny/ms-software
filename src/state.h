#pragma once

#include <taskGY521.h>

#include "tasks/motorCommands.h"
#include "tasks/robotCommands.h"
#include "tasks/taskVl53L1X.h"

struct Robot {
  Motor leftMotor;
  Motor rightMotor;
  int distances[NB_DISTANCE_SENSORS];
  ImuData imuData;
  RobotController controller;
};

extern Robot robot;