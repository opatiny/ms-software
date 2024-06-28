#pragma once

#include <state.h>

/**
 * Parameters for the initialisation of the robot.
 */
struct ControllerParams {
  int commandParameter;
  int wheelSpeedParameter;
  int linearSpeedParameter;
  int angularSpeedParameter;
  int modeParameter;
  int angleParameter;
  int obstacleDistanceParameter;
  PidInitParameters wheelsPid;
  PidInitParameters linearPid;
  PidInitParameters angularPid;
};

int getClampedCommand(int speed);
void initialiseNavigation(RobotNavigation* controller,
                          ControllerParams* params);
void robotMoveSameCommand(Robot* robot, int speed);
void robotMove(Robot* robot, int speed);
void robotStop(Robot* robot);
void robotTurnInPlace(Robot* robot, int speed);
void stopWhenObstacle(Robot* robot, int speed, int distance);
void wheelSpeedControl(Robot* robot, int speed);
void robotSpeedControl(Robot* robot, double linearSpeed, double angularSpeed);