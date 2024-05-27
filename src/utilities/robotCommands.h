#pragma once

#include "../state.h"

/**
 * Parameters for the initialisation of the robot.
 */
struct ControllerParams {
  int commandParameter;
  int modeParameter;
  int angleParameter;
  int obstacleDistanceParameter;
  PidParams wheelsParams;
};

int getClampedSpeed(int speed);
void initialiseController(RobotController* controller,
                          ControllerParams* params);
void robotMove(Robot* robot, int speed);
void robotStop(Robot* robot);
void robotTurnInPlace(Robot* robot, int speed);
void stopWhenObstacle(Robot* robot, int speed, int distance);
void robotMoveStraight(Robot* robot, int speed);