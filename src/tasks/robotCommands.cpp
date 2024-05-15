#include <utilities/params.h>

#include "../state.h"
#include "motorCommands.h"
#include "robotCommands.h"
#include "taskRobotMove.h"

void initialiseController(RobotController* controller,
                          ControllerParams* params) {
  // setup the motor parameters
  controller->speedParameter = params->speedParameter;
  controller->modeParameter = params->modeParameter;
  controller->angleParameter = params->angleParameter;
  controller->previousMode = ROBOT_STOP;
  controller->currentSpeed = 0;

  // initally stop the robot
  setParameter(controller->modeParameter, ROBOT_STOP);

  // initialise robot parameters
  setParameter(controller->speedParameter, 100);
  setParameter(controller->angleParameter, 90);  // degrees
  setParameter(controller->rampStepParameter,
               1);  // set time delay for ramps in ms
}

void robotMove(Robot* robot, int speed) {
  speedRamp(&robot->leftMotor, speed, robot->controller.rampStepParameter);
  speedRamp(&robot->rightMotor, speed, robot->controller.rampStepParameter);
  robot->controller.currentSpeed = speed;
}

void robotStop(Robot* robot) {
  stopMotor(&robot->leftMotor);
  stopMotor(&robot->rightMotor);
  robot->controller.currentSpeed = 0;
}

void robotTurnInPlace(Robot* robot, int speed) {
  speedRamp(&robot->leftMotor, speed, robot->controller.rampStepParameter);
  speedRamp(&robot->rightMotor, -speed, robot->controller.rampStepParameter);
}

void stopWhenObstacle(Robot* robot, int speed, int distance) {
  robotMove(robot, speed);
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    if (robot->distances[i] < distance) {
      robotStop(robot);
      break;
    }
  }
}