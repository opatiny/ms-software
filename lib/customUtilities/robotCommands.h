#pragma once

#include <state.h>

enum ControllerModes {
  CONTROLLER_OFF = 0,
  CONTROLLER_ON = 1,
};

/**
 * Parameters for the initialisation of the robot.
 */
struct ControllerParams {
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