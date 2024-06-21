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
  PidSerialParameters pidParams;
};

int getClampedSpeed(int speed);
void initialiseController(RobotNavigation* controller,
                          ControllerParams* params);
void robotMoveSameCommand(Robot* robot, int speed);
void robotMove(Robot* robot, int speed);
void robotStop(Robot* robot);
void robotTurnInPlace(Robot* robot, int speed);
void stopWhenObstacle(Robot* robot, int speed, int distance);
void robotMoveStraight(Robot* robot, int speed);