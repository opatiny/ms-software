#pragma once

#include <taskGY521.h>

#include "tasks/motorCommands.h"
#include "tasks/taskVl53L1X.h"

struct State {
  Motor leftMotor;
  Motor rightMotor;
  int distances[NB_DISTANCE_SENSORS];
  ImuData imuData;
};

extern State state;