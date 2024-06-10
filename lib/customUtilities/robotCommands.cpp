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
  setParameter(controller->commandParameter, 110);
  setParameter(controller->speedParameter, 400);
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
  robotMove(robot, speed);
}

void robotMoveStraight(Robot* robot, int speed) {
  // speed difference between the two wheels in rpm
  double errorRpm =
      robot->odometry.leftWheelSpeed - robot->odometry.rightWheelSpeed;
  double correction =
      getNewPidValue(&robot->controller.wheelsController, errorRpm);

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      millis() - moveStraightDebugTime > MOVE_STRAIGHT_DEBUG_DELAY) {
    Serial.print("Error: ");
    Serial.print(errorRpm);
    Serial.print(", Correction: ");
    Serial.print(correction);
    Serial.print(", Left speed rpm: ");
    Serial.print(robot->odometry.leftWheelSpeed);
    Serial.print(", Right speed rpm: ");
    Serial.print(robot->odometry.rightWheelSpeed);
    Serial.print(", Left cmd: ");
    Serial.print(robot->controller.wheelsCommands.leftSpeed);
    Serial.print(", Right speed cmd: ");
    Serial.println(robot->controller.wheelsCommands.rightSpeed);
    moveStraightDebugTime = millis();
  }

  int newLeftCmd =
      getClampedSpeed(robot->controller.wheelsCommands.leftSpeed - correction);
  int newRightCmd =
      getClampedSpeed(robot->controller.wheelsCommands.rightSpeed + correction);
  speedRamp(&robot->leftMotor, newLeftCmd, robot->controller.rampStep);
  speedRamp(&robot->rightMotor, newRightCmd, robot->controller.rampStep);

  robot->controller.wheelsCommands.leftSpeed = newLeftCmd;
  robot->controller.wheelsCommands.rightSpeed = newRightCmd;
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

void wheeSpeedController(Motor* motor, Encoder* encoder, PidController* pid) {
  double errorRpm =
      robot->odometry.leftWheelSpeed - robot->odometry.rightWheelSpeed;
  double correction =
      getNewPidValue(&robot->controller.wheelsController, errorRpm);
}