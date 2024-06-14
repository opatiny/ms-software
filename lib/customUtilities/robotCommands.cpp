#include <timeUtilities.h>
#include <utilities/params.h>

#include "motorCommands.h"
#include "robotCommands.h"
#include "robotModes.h"

int getClampedSpeed(int speed);
void wheelSpeedController(Motor* motor, Encoder* encoder, PidController* pid);
void printPidDebug(PidController* pid, Motor* motor);

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
  initialisePidController(&controller->leftSpeedController, &params->pidParams);
  initialisePidController(&controller->rightSpeedController,
                          &params->pidParams);

  // initally stop the robot
  setParameter(controller->modeParameter,
               ROBOT_EACH_WHEEL);  // todo: change back to ROBOT_STOP

  // initialise robot parameters
  setParameter(controller->commandParameter, 150);
  // setParameter(controller->speedParameter, 300);
  setParameter(controller->angleParameter, 90);  // degrees
  setParameter(controller->obstacleDistanceParameter,
               150);  // distance in mm
}

/**
 * Move the robot by applying the same command on both
 * wheels. There is no regulation or correction of the speed.
 * @param robot The robot structure.
 * @param command The speed command for the wheels.
 */
void robotMoveSameCommand(Robot* robot, int command) {
  if (command != robot->controller.currentCommand) {
    robot->controller.currentCommand = command;
  }
  updateMotors(robot, command, command, getParameter(PARAM_MOTOR_ACC_DURATION));

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->controller.previousMode != ROBOT_MOVE_SAME_COMMAND) {
    Serial.println("Robot move same command");
  }
}

/**
 * Move the robot in a straight line at a given speed in rpm.
 * This function doesn't use a PID controller, but linearizes the speed based on
 * the calibration between rpm speed and command.
 * @param robot The robot structure.
 * @param speed The desired speed for the wheels in rpm.
 */
void robotMove(Robot* robot, int speed) {
  if (speed != robot->controller.currentSpeed) {
    robot->controller.currentSpeed = speed;
  }
  int leftCommand = getCommand(&robot->leftMotor.regressions, speed);
  int rightCommand = getCommand(&robot->rightMotor.regressions, speed);
  updateMotors(robot, leftCommand, rightCommand,
               getParameter(PARAM_MOTOR_ACC_DURATION));
  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->controller.previousMode != ROBOT_MOVE) {
    Serial.println("Robot move same speed on both wheels");
  }
}

/**
 * @brief Stop the robot.
 */
void robotStop(Robot* robot) {
  stopMotors(robot);
  robot->controller.currentCommand = 0;
}

/**
 * Turn the robot in place by applying opposite speeds on the wheels.
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

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->controller.previousMode != ROBOT_TURN_IN_PLACE) {
    Serial.println("Robot turn in place");
  }
}

/**
 * Stop the robot when an obstacle is detected and make it move again
 * when obsacle is removed.
 * @param robot The robot structure.
 * @param speed The speed at which the robot should move in rpm.
 * @param distance The minimum distance to the obstacle before stopping.
 */
void stopWhenObstacle(Robot* robot, int speed, int distance) {
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    if (robot->distances[i] < distance) {
      robotStop(robot);
      return;
    }
  }
  robotMoveStraight(robot, speed);
  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->controller.previousMode != ROBOT_STOP_OBSTACLE) {
    Serial.println("Robot stop obstacle");
  }
}

/**
 * Move the robot straight at a given speed in rpm using a PID controller.
 * @param robot The robot structure.
 * @param speed The desired speed for the wheels in rpm.

*/
void robotMoveStraight(Robot* robot, int speed) {
  if (robot->controller.clearControllers) {  // todo: check this condition
    clearController(&robot->controller.leftSpeedController);
    clearController(&robot->controller.rightSpeedController);
    robot->controller.clearControllers = 0;
  }

  robot->controller.leftSpeedController.targetValue = speed;
  robot->controller.rightSpeedController.targetValue = speed;

  wheelSpeedController(&robot->leftMotor, &robot->leftEncoder,
                       &robot->controller.leftSpeedController);
  wheelSpeedController(&robot->rightMotor, &robot->rightEncoder,
                       &robot->controller.rightSpeedController);

  // printPidDebug(&robot->controller.leftSpeedController, &robot->leftMotor);

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->controller.previousMode != ROBOT_MOVE_STRAIGHT &&
      robot->controller.previousMode != ROBOT_STOP_OBSTACLE) {
    Serial.println("Robot move straight with PID");
  }

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
    // time, targetSpeed, leftSpeed, leftCommand, rightSpeed, rightCommand
    Serial.print(getSeconds(), 3);
    Serial.print(", ");
    Serial.print(speed);
    Serial.print(", ");
    Serial.print(robot->leftMotor.wheelSpeed);
    Serial.print(", ");
    Serial.print(robot->leftMotor.currentCommand);
    Serial.print(", ");
    Serial.print(robot->rightMotor.wheelSpeed);
    Serial.print(", ");
    Serial.println(robot->rightMotor.currentCommand);
  }
}

/**
 * Get the clamped speed value between the minimum and maximum speed
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

/**
 * @brief Control the speed of a wheel using a PID controller. The desired
 target speed in rpm is in the pid structure..
 * @param motor The motor structure.
 * @param encoder The encoder structure.
 * @param pid The PID controller structure.

*/
void wheelSpeedController(Motor* motor, Encoder* encoder, PidController* pid) {
  double errorRpm = motor->wheelSpeed - pid->targetValue;
  double correction = getNewPidValue(pid, errorRpm);
  int newCmd = getClampedSpeed(motor->currentCommand - correction);

  updateMotor(motor, newCmd,
              1);  // duration is 1 because movement must be as fast as possible
}

void printPidDebug(PidController* pid, Motor* motor) {
  Serial.print("Current: ");
  Serial.print(motor->wheelSpeed);
  Serial.print(", Error: ");
  Serial.print(pid->previousError);
  Serial.print(", Correction: ");
  Serial.print(pid->correction);
  Serial.print(", New command value: ");
  Serial.println(motor->currentCommand);
}