#include "../hardwareProperties.h"

#include "kinematics.h"

/**
 * @brief Convert nb of encoder counts to an angle in degrees.
 * todo: Is the return type int a problem?
 */
int countsToAngle(int counts) {
  return counts * 360 / (12 * GEAR_RATIO);
}

/**
 * @brief Convert an angle in degrees to nb of encoder counts.
 */
int angleToCounts(int angle) {
  return angle * 12 * GEAR_RATIO / 360;
}

UnicycleSpeed diffToUnicycleSpeed(DiffSpeed diffSpeed) {
  int L = WHEEL_BASE;

  double v = (diffSpeed.left + diffSpeed.right) / 2;
  double rho = L * v / (diffSpeed.right - diffSpeed.left);
  return {v, rho};
}

DiffSpeed unicycleToDiffSpeed(UnicycleSpeed unicycleSpeed) {
  int L = WHEEL_BASE;
  double vl = unicycleSpeed.v * (1 - L / (4 * unicycleSpeed.rho));
  double vr = unicycleSpeed.v * (1 + L / (4 * unicycleSpeed.rho));
  return {vl, vr};
}