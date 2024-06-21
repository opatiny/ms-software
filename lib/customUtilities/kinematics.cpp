#include <Arduino.h>

#include "kinematics.h"
#include "motorProperties.h"

/**
 * @brief Convert nb of encoder counts to an angle in degrees.
 */
double countsToAngle(int counts) {
  return counts * 360.0 / (ENCODER_COUNTS_PER_REV * GEAR_RATIO);
}

/**
 * @brief Convert an angle in degrees to nb of encoder counts.
 */
int angleToCounts(int angle) {
  return angle * ENCODER_COUNTS_PER_REV * GEAR_RATIO / 360;
}

UnicycleSpeed diffToUnicycleSpeed(DiffSpeed diffSpeed) {
  int L = WHEEL_BASE;
  double v = (diffSpeed.left + diffSpeed.right) / 2;
  double omega = (diffSpeed.right - diffSpeed.left) / L;
  return {v, omega};
}

DiffSpeed unicycleToDiffSpeed(UnicycleSpeed unicycleSpeed) {
  int L = WHEEL_BASE;
  double vl = unicycleSpeed.v - L * unicycleSpeed.omega / 2;
  double vr = unicycleSpeed.v + L * unicycleSpeed.omega / 2;
  return {vl, vr};
}

double computeCurvature(UnicycleSpeed unicycleSpeed) {
  if (unicycleSpeed.omega == 0) {
    return 0;
  }
  return unicycleSpeed.v / unicycleSpeed.omega;
}

/**
 * @brief Compute the wheel speed in rpm from the encoder counts difference
 * and the time elapsed since the last update.
 * @param counts The number of encoder counts since the last update.
 * @param dt The time elapsed since the last update in seconds.
 */
double computeWheelRpm(int counts, double dt) {
  double angle = countsToAngle(counts);
  if (dt == 0) {
    Serial.println("computeWheelRpm dt is 0");
  }
  double degSec = angle / dt;
  return degSec / 6.0;
}

/**
 * @brief Convert speed from counts/us to rpm.
 */
double getLowSpeedRpm(double lowSpeed) {
  return lowSpeed / (ENCODER_COUNTS_PER_REV * GEAR_RATIO) * 60 * 1000000;
}