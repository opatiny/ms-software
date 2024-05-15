#pragma once

#include "../state.h"

/**
 * Parameters for the initialisation of the robot.
 */
struct ControllerParams {
  int speedParameter;
  int modeParameter;
  int angleParameter;
  int rampStepParameter;
  int obstacleDistanceParameter;
};

void initialiseController(RobotController* controller,
                          ControllerParams* params);
void robotMove(Robot* robot, int speed);
void robotStop(Robot* robot);
void robotTurnInPlace(Robot* robot, int speed);
void stopWhenObstacle(Robot* robot, int speed, int distance);