#pragma once

#include <regressions.h>

#define SPEED_CALIBRATION_DELAY 1000
#define COMMAND_STEP 30
#define MIN_MOTOR_COMMAND -255
#define MAX_MOTOR_COMMAND 255
#define CALIBRATION_SPEED_LIMIT 550  // rpm

enum CalibrationModes { CALIBRATION_OFF, CALIBRATION_ON };

/**
 * Data for the speed calibration.
 * - commands: array of motor commands
 * - leftSpeeds: array of left wheel speeds in rpm
 * - rightSpeeds: array of right wheel speeds in rpm
 * - index: index of the current value in the arrays
 * - command: current motor command
 */
struct CalibrationData {
  DataArray commands;
  DataArray leftSpeeds;
  DataArray rightSpeeds;
  int index;
  int command;
};

void initialiseCalibrationData(CalibrationData* data);
void wheelSpeedCalibration(CalibrationData* data,
                           Regressions* leftRegressions,
                           Regressions* rightRegressions);
void clearCalibrationData(CalibrationData* data);