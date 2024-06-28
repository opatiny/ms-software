#pragma once

/**
 * The linear speed of the robot in m/s and the angular speed in rad/s.
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
double getLowSpeedRpm(double lowSpeed);