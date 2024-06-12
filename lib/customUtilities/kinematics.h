#pragma once

/**
 * Speed of the center of the robot in m/s and curvature radius rho.
 * - turning left: positive rho
 * - turning right: negative rho
 */
struct UnicycleSpeed {
  double v;
  double omega;
};

/**
 * Speed of the left and right wheels in m/s.
 */
struct DiffSpeed {
  double left;
  double right;
};

double countsToAngle(int counts);
int angleToCounts(int angle);
UnicycleSpeed diffToUnicycleSpeed(DiffSpeed diffSpeed);
DiffSpeed unicycleToDiffSpeed(UnicycleSpeed unicycleSpeed);
double computeCurvature(UnicycleSpeed unicycleSpeed);
double computeWheelRpm(int counts, double dt);