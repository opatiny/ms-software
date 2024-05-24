#pragma once

/**
 * Speed of the center of the robot in m/s and curvature radius rho.
 * - turning left: positive rho
 * - turning right: negative rho
 */
struct UnicycleSpeed {
  double v;
  double rho;
};

/**
 * Speed of the left and right wheels in m/s.
 */
struct DiffSpeed {
  double left;
  double right;
};

float countsToAngle(int counts);
int angleToCounts(int angle);
UnicycleSpeed diffToUnicycleSpeed(DiffSpeed diffSpeed);
DiffSpeed unicycleToDiffSpeed(UnicycleSpeed unicycleSpeed);
float computeWheelRpm(int counts, int dt);