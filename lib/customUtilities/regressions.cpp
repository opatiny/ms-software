#include "regressions.h"
#include <curveFitting.h>

/**
 * @brief Find the coefficients of the polynomial regressions for the negative
 * and positive parts of the curve. We use this function to find the
 * relationship between motor speed and command. The degree of the polynoms is
 * hardcoded to 4 in variable POLYNOM_DEGREE.
 * @param regessions: The structure containing the coefficients of the two
 * polynoms.
 * @param commands: The commands data.
 * @param speeds: The speeds data.
 * @param speedLimit: The maximm absolute value of speeds to consider for the
 * regressions.
 */
void getRegressions(Regressions* regessions,
                    DataArray commands,
                    DataArray speeds,
                    double speedLimit) {
  // do not consider all points outside of the commands limit
  int start = findMinIndex(speeds, -speedLimit);
  int end = findMaxIndex(speeds, speedLimit);

  int minZero = findMinZero(speeds);
  int maxZero = findMaxZero(speeds);

  Serial.print("start: ");
  Serial.print(start);
  Serial.print(", end: ");
  Serial.print(end);
  Serial.print(", minZero: ");
  Serial.print(minZero);
  Serial.print(", maxZero: ");
  Serial.println(maxZero);

  int negLength = minZero - start + 1;
  int posLength = end - maxZero + 1;

  Serial.print("negLength: ");
  Serial.print(negLength);
  Serial.print(", posLength: ");
  Serial.println(posLength);

  // find arrays for the polynomial regressions
  double* commandsNeg = commands + start;
  double* speedsNeg = speeds + start;
  double* commandsPos = commands + maxZero;
  double* speedsPos = speeds + maxZero;

  int negError = fitCurve(POLYNOM_DEGREE, negLength, speedsNeg, commandsNeg,
                          NB_COEFF, regessions->pNeg);
  int posError = fitCurve(POLYNOM_DEGREE, posLength, speedsPos, commandsPos,
                          NB_COEFF, regessions->pPos);
}

/**
 * @brief Find the index of the first value in the array that is non null and
 * greater or equal to the given value. This function only works when data
 * contains negative values at the beginning.
 */
int findMinIndex(DataArray dataArray, double minValue) {
  int index = 0;
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    if (dataArray[i] >= minValue & dataArray[i] != 0) {
      index = i;
      break;
    }
  }
  return index;
}

/**
 * @brief Find the index of the last value in the array that non null, or that
 * is smaller than the given value.
 */
int findMaxIndex(DataArray dataArray, double maxValue) {
  int index = 0;
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    if (dataArray[i] > maxValue ||
        (dataArray[i] == 0 && dataArray[i - 1] > 0)) {
      break;
    }
    index = i;
  }
  return index;
}

int findMinZero(DataArray dataArray) {
  int index = 0;
  int previousValue = 0;
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    index = i;
    if (dataArray[i] == 0 && previousValue < 0) {
      break;
    }

    previousValue = dataArray[i];
  }
  return index;
}

int findMaxZero(DataArray dataArray) {
  int index = 0;
  int previousValue = 0;
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    if (dataArray[i] > 0 && previousValue == 0) {
      break;
    }
    index = i;
    previousValue = dataArray[i];
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

double polyVal(double* polynom, int degree, double commands) {
  double result = 0;
  for (int i = 0; i < degree + 1; i++) {
    result += polynom[i] * pow(commands, i);
  }
  return result;
}