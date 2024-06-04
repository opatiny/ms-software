#pragma once

#include <regressions.h>

#define SPEED_CALIBRATION_DELAY 1000
#define COMMAND_STEP 1
#define MIN_MOTOR_COMMAND -255
#define MAX_MOTOR_COMMAND 255
#define CALIBRATION_SPEED_LIMIT 550  // rpm

/**
 * Data for the speed calibration.
 * - commands: array of motor commands
 * - leftSpeeds: array of left wheel speeds in rpm
 * - rightSpeeds: array of right wheel speeds in rpm
 * - index: index of the current value in the arrays
 * - previousTime: time of the previous calibration step
 * - command: current motor command
 */
struct CalibrationData {
  DataArray commands;
  DataArray leftSpeeds;
  DataArray rightSpeeds;
  int index;
  int previousTime;
  int command;
};

void initialiseCalibrationData(CalibrationData* data);
void wheelSpeedCalibration(CalibrationData* data);
void clearCalibrationData(CalibrationData* data);