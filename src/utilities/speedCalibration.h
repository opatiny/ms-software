#pragma once

#define SPEED_CALIBRATION_DELAY 1000
#define COMMAND_STEP 1
#define MIN_MOTOR_COMMAND -255
#define MAX_MOTOR_COMMAND 255

#define CALIBRATION_MAX_NB_VALUES 512

#define POLYNOM_DEGREE 4

typedef int Polynom[POLYNOM_DEGREE + 1];
typedef int MinMaxIndices[2];
typedef double CalibrationArray[CALIBRATION_MAX_NB_VALUES];

struct CalibrationData {
  CalibrationArray commands;
  CalibrationArray leftSpeeds;
  CalibrationArray rightSpeeds;
  int index;
  int previousTime;
};

struct Regressions {
  Polynom pNeg;
  Polynom pPos;
};