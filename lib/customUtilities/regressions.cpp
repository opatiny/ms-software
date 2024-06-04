#include "regressions.h"
#include <curveFitting.h>

/**
 * @brief Find the coefficients of the polynomial regressions for the negative
 * and positive parts of the curve. We use this function to find the
 * relationship between motor speed and command. The degree of the polynoms is
 * hardcoded to 4 in variable POLYNOM_DEGREE.
 * @param regessions: The structure containing the coefficients of the two
 * polynoms.
 * @param x: The x data.
 * @param y: The y data.
 * @param xLimit: The maximm absolute value of x to consider for the
 * regressions.
 */
void getRegressions(Regressions* regessions,
                    DataArray x,
                    DataArray y,
                    double xLimit) {
  // do not consider all points outside of the x limit
  int start = findMinIndex(x, -xLimit);
  int end = findMaxIndex(x, xLimit);

  int minZero = findMinIndex(x, 0);
  int maxZero = findMaxIndex(x, 0);

  Serial.print("start: ");
  Serial.print(start);
  Serial.print(", end: ");
  Serial.print(end);
  Serial.print(", minZero: ");
  Serial.print(minZero);
  Serial.print(", maxZero: ");
  Serial.println(maxZero);

  int negLength = minZero - start;
  int posLength = end - maxZero;

  // find arrays for the polynomial regressions
  double* xNeg = x + start;
  double* yNeg = y + start;
  double* xPos = x + maxZero;
  double* yPos = y + maxZero;

  int negError = fitCurve(POLYNOM_DEGREE, negLength, xNeg, yNeg, NB_COEFF,
                          regessions->pNeg);
  int posError = fitCurve(POLYNOM_DEGREE, posLength, xPos, yPos, NB_COEFF,
                          regessions->pPos);
}

/**
 * @brief Find the index of the first value in the array that is greater or
 * equal to the given value.
 * This function only works when data contains negative values at the beginning.
 */
int findMinIndex(DataArray dataArray, double minValue) {
  int index = 0;
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    if (dataArray[i] >= minValue) {
      index = i;
      break;
    }
  }
  return index;
}

/**
 * @brief Find the index of the last value in the array that is less or equal to
 * the given value.
 */
int findMaxIndex(DataArray dataArray, double maxValue) {
  int index = 0;
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    if (dataArray[i] > maxValue) {
      break;
    }
    index = i;
  }
  return index;
}

int getCommand(Regressions* reg, double speed) {
  double command = 0;
  if (speed < 0) {
    polyVal(reg->pNeg, POLYNOM_DEGREE, speed);
  } else if (speed > 0) {
    polyVal(reg->pPos, POLYNOM_DEGREE, speed);
  }
  return abs(command);
}

double polyVal(double* polynom, int degree, double x) {
  double result = 0;
  for (int i = 0; i < degree + 1; i++) {
    result += polynom[i] * pow(x, i);
  }
  return result;
}