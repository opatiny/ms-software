#include <utilities/params.h>

#include "../state.h"
#include "../tasks/taskRobotMove.h"
#include "motorCommands.h"
#include "robotCommands.h"

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
  controller->speedParameter = params->speedParameter;
  controller->modeParameter = params->modeParameter;
  controller->angleParameter = params->angleParameter;
  controller->obstacleDistanceParameter = params->obstacleDistanceParameter;
  controller->previousMode = ROBOT_STOP;
  controller->rampStep = 1;  // 1ms delay between each speed increment

  // initialize PID
  initialisePidController(&controller->wheelsController, &params->wheelsParams);

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
    robot->controller.wheelsCommands.leftSpeed = speed;
    robot->controller.wheelsCommands.rightSpeed = speed;
  } else {
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

    int newLeftCmd = getClampedSpeed(
        robot->controller.wheelsCommands.leftSpeed - correction);
    int newRightCmd = getClampedSpeed(
        robot->controller.wheelsCommands.rightSpeed + correction);
    speedRamp(&robot->leftMotor, newLeftCmd, robot->controller.rampStep);
    speedRamp(&robot->rightMotor, newRightCmd, robot->controller.rampStep);

    robot->controller.wheelsCommands.leftSpeed = newLeftCmd;
    robot->controller.wheelsCommands.rightSpeed = newRightCmd;
  }
}

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