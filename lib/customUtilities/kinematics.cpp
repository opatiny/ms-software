#include <Arduino.h>

#include "../../src/hardwareProperties.h"
#include "kinematics.h"

/**
 * @brief Convert nb of encoder counts to an angle in degrees.
 * todo: Is the return type int a problem?
 */
float countsToAngle(int counts) {
  return counts * 360 / (ENCODER_COUNTS_PER_REV * GEAR_RATIO);
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
  return unicycleSpeed.v / unicycleSpeed.omega;
}

/**
 * @brief Compute the wheel speed in rpm from the encoder counts difference and
 * the time elapsed since the last update.
 * @param counts The number of encoder counts since the last update.
 * @param dt The time elapsed since the last update in seconds.
 */
float computeWheelRpm(int counts, float dt) {
  float angle = countsToAngle(counts);
  float degSec = angle / dt;
  return degSec * 60 / 360;
}