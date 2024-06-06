#pragma once

#include <regressions.h>

#define SPEED_CALIBRATION_DELAY 1000
#define MIN_MOTOR_COMMAND -255
#define MAX_MOTOR_COMMAND 255
#define CALIBRATION_SPEED_LIMIT 550  // rpm
#define CALIBRATION_TEST_SPEED_STEP 10

enum CalibrationModes { CALIBRATION_OFF, CALIBRATION_ON, CALIBRATION_TEST };

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
/**
 * Structure to test the speed calibration.
 * - speedStep: speed step in rpm between each iteration
 * - speed: current speed in rpm
 * Min and max speeds are defined by CALIBRATION_SPEED_LIMIT.
 */
struct TestCalibrationData {
  int speedStep;
  int speed;
};

void initialiseCalibrationData(CalibrationData* data);
void wheelSpeedCalibration(CalibrationData* data,
                           Regressions* leftRegressions,
                           Regressions* rightRegressions);
void clearCalibrationData(CalibrationData* data);

void initialiseTestCalibrationData(TestCalibrationData* data);
void testCalibration(Robot* robot, TestCalibrationData* data);
void clearTestCalibrationData(TestCalibrationData* data);