#include <utilities/params.h>

#include "robotCommands.h"

#include "../state.h"
#include "taskRobotMove.h"

void initialiseRobot(Robot* robot, RobotParams* params) {
  // setup the motor parameters
  robot->speedParameter = params->speedParameter;
  robot->modeParameter = params->modeParameter;
  robot->angleParameter = params->angleParameter;
  robot->previousMode = ROBOT_STOP;
  robot->currentSpeed = 0;

  // initally stop the robot
  setParameter(robot->modeParameter, ROBOT_STOP);

  // initialise motor parameters
  setParameter(robot->speedParameter, 100);
  setParameter(robot->angleParameter, 90);  // degrees
}

void robotMove(int speed) {
  speedRamp(&state.leftMotor, speed);
  speedRamp(&state.rightMotor, speed);
  state.robot.currentSpeed = speed;
}

void robotStop() {
  stopMotor(&state.leftMotor);
  stopMotor(&state.rightMotor);
  state.robot.currentSpeed = 0;
}

void robotTurnInPlace(int speed) {
  speedRamp(&state.leftMotor, speed);
  speedRamp(&state.rightMotor, -speed);
}
