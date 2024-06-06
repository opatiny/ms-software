#include <utilities/params.h>

#include "motorCommands.h"
#include "robotCommands.h"
#include "robotModes.h"

#define MOVE_STRAIGHT_DEBUG_DELAY 500
int moveStraightDebugTime = millis();

int getClampedSpeed(int speed);

/**
 * @brief Initialise the robot controller by giving the desired default values
 * to the parameters.
 */
void initialiseController(RobotController* controller,
                          ControllerParams* params) {
  // setup the motor parameters
  controller->commandParameter = params->commandParameter;
  controller->speedParameter = params->speedParameter;
  controller->modeParameter = params->modeParameter;
  controller->angleParameter = params->angleParameter;
  controller->obstacleDistanceParameter = params->obstacleDistanceParameter;
  controller->previousMode = ROBOT_STOP;

  // initialize PID
  initialisePidController(&controller->angularPid, &params->wheelsParams);

  // initally stop the robot
  setParameter(controller->modeParameter,
               ROBOT_EACH_WHEEL);  // todo: change back to ROBOT_STOP

  // initialise robot parameters
  // setParameter(controller->commandParameter, 70);
  setParameter(controller->angleParameter, 90);  // degrees
  setParameter(controller->obstacleDistanceParameter,
               150);  // distance in mm
}

/**
 * @brief Move the robot by applying the same command on both
 * wheels. There is no regulation or correction of the speed.
 * @param robot The robot structure.
 * @param command The speed command for the wheels
 */
void robotMoveSameCommand(Robot* robot, int command) {
  if (command != robot->controller.currentCommand) {
    robot->controller.currentCommand = command;
  }
  updateMotors(robot, command, command, getParameter(PARAM_MOTOR_ACC_DURATION));
}

/**
 * @brief Move the robot at a given speed in rpm. The command of each wheel is
 * computed based on the regressions of the speed calibration.
 * @param robot The robot structure.
 * @param speed The desired speed for the wheels.
 */
void robotMove(Robot* robot, int speed) {
  if (speed != robot->controller.currentSpeed) {
    robot->controller.currentSpeed = speed;
  }
  int leftCommand = getCommand(&robot->leftMotor.regressions, speed);
  int rightCommand = getCommand(&robot->rightMotor.regressions, speed);
  updateMotors(robot, leftCommand, rightCommand,
               getParameter(PARAM_MOTOR_ACC_DURATION));
}

/**
 * @brief Stop the robot.
 */
void robotStop(Robot* robot) {
  stopMotors(robot);
  robot->controller.currentCommand = 0;
}

/**
 * @brief Turn the robot in place by applying opposite speeds on the wheels.
 * @param robot The robot structure.
 * @param speed The speed of the wheels in rpm, defines how fast the robot wil
 * turn. Positive values turn the robot clockwise, negative values turn the
 * robot counter-clockwise.
 */
void robotTurnInPlace(Robot* robot, int speed) {
  if (speed != robot->controller.currentSpeed) {
    robot->controller.currentSpeed = 0;
  }
  int leftCommand = getCommand(&robot->leftMotor.regressions, speed);
  int rightCommand = getCommand(&robot->rightMotor.regressions, -speed);
  updateMotors(robot, leftCommand, rightCommand,
               getParameter(PARAM_MOTOR_ACC_DURATION));
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
  robotMoveSameCommand(robot, speed);
}

/**
 * @brief Move the robot in a straight line at a given speed in rpm.
 * This function doesn't use a PID controller, but linearizes the speed based on
 * the calibration between rpm speed and command.
 * @param robot The robot structure.
 * @param speed Desired linear speed in rpm.
 */
void robotMoveStraight(Robot* robot, int speed) {}

/**
 * @brief Get the clamped speed value between the minimum and maximum speed
 * values.
 * @param speed The speed to clamp.
 * @return The clamped speed value.
 */
int getClampedSpeed(int speed) {
  int rounded = round(speed);
  if (speed > MAX_SPEED_COMMAND) {
    return MAX_SPEED_COMMAND;
  } else if (speed < MIN_SPEED_COMMAND) {
    return MIN_SPEED_COMMAND;
  }
  return speed;
}