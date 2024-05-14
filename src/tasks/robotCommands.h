#pragma once

struct Robot {
  int speedParameter;
  int angleParameter;
  int distanceParameter;
  int modeParameter;
  int currentSpeed;
  int previousMode;
};

/**
 * Parameters for the initialisation of the robot.
 */
struct RobotParams {
  int speedParameter;
  int modeParameter;
  int angleParameter;
};

void robotMove(int speed);
void robotStop();
void robotTurnInPlace(int speed);