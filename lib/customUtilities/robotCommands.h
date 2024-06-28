#pragma once

#include <state.h>

/**
 * Parameters for the initialisation of the robot.
 */
struct ControllerParams {
  int commandParameter;
  int speedParameter;
  int modeParameter;
  int angleParameter;
  int obstacleDistanceParameter;
  PidSerialParameters wheelsPid;
  PidSerialParameters linearPid;
  PidSerialParameters angularPid;
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
void robotSpeedControl(Robot* robot, int linearSpeed, int angularSpeed);