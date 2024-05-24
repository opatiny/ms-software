#include <utilities/params.h>

#include "../state.h"
#include "../tasks/taskRobotMove.h"
#include "motorCommands.h"
#include "robotCommands.h"

/**
 * @brief Initialise the robot controller by giving the desired default values
 * to the parameters.
 */
void initialiseController(RobotController* controller,
                          ControllerParams* params) {
  // setup the motor parameters
  controller->speedParameter = params->speedParameter;
  controller->modeParameter = params->modeParameter;
  controller->angleParameter = params->angleParameter;
  controller->obstacleDistanceParameter = params->obstacleDistanceParameter;
  controller->previousMode = ROBOT_STOP;
  controller->currentSpeed = 0;
  controller->rampStep = 1;  // 1ms delay between each speed increment

  // initally stop the robot
  setParameter(controller->modeParameter, ROBOT_STOP);

  // initialise robot parameters
  // setParameter(controller->speedParameter, 70);
  setParameter(controller->angleParameter, 90);  // degrees
  setParameter(controller->obstacleDistanceParameter,
               150);  // distance in mm
  controller->rampStep = 1;
}

/**
 * @brief Move the robot at a given speed by applying the same command on both
 * wheels. There is no regulation of the speed.
 * @param robot The robot structure.
 * @param speed The speed command for the wheels
 */
void robotMove(Robot* robot, int speed) {
  if (speed != robot->controller.currentSpeed) {
    speedRamp(&robot->leftMotor, speed, robot->controller.rampStep);
    speedRamp(&robot->rightMotor, speed, robot->controller.rampStep);
    robot->controller.currentSpeed = speed;
  }
}

/**
 * @brief Stop the robot.
 */
void robotStop(Robot* robot) {
  stopMotor(&robot->leftMotor);
  stopMotor(&robot->rightMotor);
  robot->controller.currentSpeed = 0;
}

/**
 * @brief Turn the robot in place by applying opposite commands on the wheels.
 */
void robotTurnInPlace(Robot* robot, int speed) {
  speedRamp(&robot->leftMotor, speed, robot->controller.rampStep);
  speedRamp(&robot->rightMotor, -speed, robot->controller.rampStep);
}

/**
 * @brief Stop the robot when an obstacle is detected and make it move again
 * when obsacle is removed.
 * @param robot The robot structure.
 * @param speed The speed at which the robot should move.
 * @param distance The minimum distance to the obstacle before stopping.
 */
void stopWhenObstacle(Robot* robot, int speed, int distance) {
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    if (robot->distances[i] < distance) {
      robotStop(robot);
      return;
    }
  }
  robotMove(robot, speed);
}

void robotMoveStraight(Robot* robot, int speed) {
  if (speed != robot->controller.currentSpeed) {
    speedRamp(&robot->leftMotor, speed, robot->controller.rampStep);
    speedRamp(&robot->rightMotor, speed, robot->controller.rampStep);
    robot->controller.currentSpeed = speed;
  }
}