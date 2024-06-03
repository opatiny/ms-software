#pragma once

#define POLYNOM_DEGREE 4
#define CALIBRATION_MAX_NB_VALUES 512

typedef double Polynom[POLYNOM_DEGREE + 1];

struct Regressions {
  Polynom pNeg;
  Polynom pPos;
};

typedef double DataArray[CALIBRATION_MAX_NB_VALUES];

int findMinIndex(DataArray speeds, double minValue);
int findMaxIndex(DataArray speeds, double maxValue);
void getRegressions(Regressions* regessions,
                    DataArray x,
                    DataArray y,
                    double xLimit);
double polyVal(double* polynom, int degree, double x);
int getCommand(Regressions* reg, double speed);