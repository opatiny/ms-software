#pragma once

struct RobotController {
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
struct ControllerParams {
  int speedParameter;
  int modeParameter;
  int angleParameter;
};

void initialiseController(RobotController* controller,
                          ControllerParams* params);
void robotMove(Robot* robot, int speed);
void robotStop(Robot* robot);
void robotTurnInPlace(Robot* robot, int speed);