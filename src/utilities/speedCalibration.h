#pragma once

#include <regressions.h>

#define SPEED_CALIBRATION_DELAY 1000
#define COMMAND_STEP 1
#define MIN_MOTOR_COMMAND -255
#define MAX_MOTOR_COMMAND 255

struct CalibrationData {
  DataArray commands;
  DataArray leftSpeeds;
  DataArray rightSpeeds;
  int index;
  int previousTime;
};
