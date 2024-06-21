#include <timeUtilities.h>
#include <utilities/params.h>

#include "../../src/tasks/taskRgbLed.h"
#include "motorCommands.h"
#include "robotCommands.h"
#include "robotModes.h"

int getClampedCommand(int command);
void wheelSpeedController(Motor* motor, Encoder* encoder, PidController* pid);
void printPidDebug(PidController* pid, Motor* motor);

/**
 * @brief Initialise the robot navigation by giving the desired default values
 * to the parameters.
 */
void initialiseController(RobotNavigation* navigation,
                          ControllerParams* params) {
  // setup the motor parameters
  navigation->commandParameter = params->commandParameter;
  navigation->speedParameter = params->speedParameter;
  navigation->modeParameter = params->modeParameter;
  navigation->angleParameter = params->angleParameter;
  navigation->obstacleDistanceParameter = params->obstacleDistanceParameter;
  navigation->previousMode = ROBOT_STOP;

  // initialize PID of wheel speeds
  initialisePidController(&navigation->wheelsSpeedController.left,
                          &params->wheelsPid);
  initialisePidController(&navigation->wheelsSpeedController.right,
                          &params->wheelsPid);

  // initialize PID of robot speed
  initialisePidController(&navigation->robotSpeedController.linear,
                          &params->linearPid);
  initialisePidController(&navigation->robotSpeedController.angular,
                          &params->angularPid);

  // initally stop the robot
  setParameter(navigation->modeParameter,
               ROBOT_EACH_WHEEL);  // todo: change back to ROBOT_STOP

  // initialise robot parameters
  setParameter(navigation->commandParameter, 150);
  // setParameter(navigation->speedParameter, 300);
  setParameter(navigation->angleParameter, 90);  // degrees
  setParameter(navigation->obstacleDistanceParameter,
               150);  // distance in mm
}

/**
 * Move the robot by applying the same command on both
 * wheels. There is no regulation or correction of the speed.
 * @param robot The robot structure.
 * @param command The speed command for the wheels.
 */
void robotMoveSameCommand(Robot* robot, int command) {
  if (command != robot->navigation.currentCommand) {
    robot->navigation.currentCommand = command;
  }
  updateMotors(robot, command, command, getParameter(PARAM_MOTOR_ACC_DURATION));

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->navigation.previousMode != ROBOT_MOVE_SAME_COMMAND) {
    Serial.println("Robot move same command");
  }
}

/**
 * Move the robot in a straight line at a given speed in rpm.
 * This function doesn't use a PID navigation, but linearizes the speed based on
 * the calibration between rpm speed and command.
 * @param robot The robot structure.
 * @param speed The desired speed for the wheels in rpm.
 */
void robotMove(Robot* robot, int speed) {
  if (speed != robot->navigation.currentSpeed) {
    robot->navigation.currentSpeed = speed;
  }
  int leftCommand = getCommand(&robot->leftMotor.regressions, speed);
  int rightCommand = getCommand(&robot->rightMotor.regressions, speed);
  updateMotors(robot, leftCommand, rightCommand,
               getParameter(PARAM_MOTOR_ACC_DURATION));
  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->navigation.previousMode != ROBOT_MOVE) {
    Serial.println("Robot move same speed on both wheels");
  }
}

/**
 * @brief Stop the robot.
 */
void robotStop(Robot* robot) {
  stopMotors(robot);
  robot->navigation.currentCommand = 0;
}

/**
 * Turn the robot in place by applying opposite speeds on the wheels.
 * @param robot The robot structure.
 * @param speed The speed of the wheels in rpm, defines how fast the robot wil
 * turn. Positive values turn the robot clockwise, negative values turn the
 * robot counter-clockwise.
 */
void robotTurnInPlace(Robot* robot, int speed) {
  if (speed != robot->navigation.currentSpeed) {
    robot->navigation.currentSpeed = 0;
  }
  int leftCommand = getCommand(&robot->leftMotor.regressions, speed);
  int rightCommand = getCommand(&robot->rightMotor.regressions, -speed);
  updateMotors(robot, leftCommand, rightCommand,
               getParameter(PARAM_MOTOR_ACC_DURATION));

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->navigation.previousMode != ROBOT_TURN_IN_PLACE) {
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
      robot->navigation.wheelsSpeedController.clearControllers = 1;
      robotStop(robot);
      rgbLedFlags.obstacleDetected = 1;
      return;
    }
  }
  wheelSpeedControl(robot, speed);
  rgbLedFlags.obstacleDetected = 0;
  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->navigation.previousMode != ROBOT_STOP_OBSTACLE) {
    Serial.println("Robot stop obstacle");
  }
}

/**
 * Move the robot straight at a given wheel speed in rpm using PID controllers
 * on the wheels.
 * @param robot The robot structure.
 * @param speed The desired speed for the wheels in rpm.
 */
void wheelSpeedControl(Robot* robot, int speed) {
  if (robot->navigation.wheelsSpeedController
          .clearControllers) {  // todo: check this condition
    clearController(&robot->navigation.wheelsSpeedController.left);
    clearController(&robot->navigation.wheelsSpeedController.right);
    robot->navigation.wheelsSpeedController.clearControllers = 0;
    if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
      Serial.println("Clear controllers");
    }
  }

  robot->navigation.wheelsSpeedController.left.targetValue = speed;
  robot->navigation.wheelsSpeedController.right.targetValue = speed;

  wheelSpeedController(&robot->leftMotor, &robot->leftEncoder,
                       &robot->navigation.wheelsSpeedController.left);
  wheelSpeedController(&robot->rightMotor, &robot->rightEncoder,
                       &robot->navigation.wheelsSpeedController.right);

  // printPidDebug(&robot->navigation.leftSpeedController, &robot->leftMotor);

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL &&
      robot->navigation.previousMode != ROBOT_WHEEL_SPEED_CONTROL &&
      robot->navigation.previousMode != ROBOT_STOP_OBSTACLE) {
    Serial.println("Robot move straight with PID on wheels speeds");
  }

  if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
    // time, targetSpeed, leftSpeed, leftCommand, rightSpeed, rightCommand
    Serial.print(millis());
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

  vTaskDelay(1);
}

// todo: finish this function
// caution: copy of struct or pointer to struct??
/**
 * Control the robot's angular and linear speed using PID controllers.
 * @param robot The robot structure.
 * @param linearSpeed The desired linear speed in m/s.
 * @param angularSpeed The desired angular speed in rad/s.
 */
void robotSpeedControl(Robot* robot, int linearSpeed, int angularSpeed) {
  RobotSpeedController robotController = robot->navigation.robotSpeedController;

  if (robotController.clearControllers) {
    clearController(&robotController.linear);
    clearController(&robotController.angular);
    robotController.clearControllers = 0;
    if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
      Serial.println("Clear controllers");
    }
  }

  robotController.linear.targetValue = linearSpeed;
  robotController.angular.targetValue = angularSpeed;

  double linError =
      robot->odometry.speed.v - robotController.linear.targetValue;
  double linCorrection = getNewPidValue(&robotController.linear, linError);

  double angError =
      robot->odometry.speed.omega - robotController.angular.targetValue;
  double angCorrection = getNewPidValue(&robotController.angular, angError);

  double leftCorrection = linCorrection - angCorrection;
  double rightCorrection = linCorrection + angCorrection;

  double leftCommand =
      getClampedCommand(robot->leftMotor.currentCommand - leftCorrection);
  double rightCommand =
      getClampedCommand(robot->rightMotor.currentCommand - rightCorrection);

  // duration is 1 because movement must be as fast as possible
  updateMotor(&robot->leftMotor, leftCommand, 1);
  updateMotor(&robot->rightMotor, rightCommand, 1);
  vTaskDelay(1);
}

/**
 * Get the clamped command value between the minimum and maximum command
 * values.
 * @param command The command to clamp.
 * @return The clamped command value.
 */
int getClampedCommand(int command) {
  int rounded = round(command);
  if (command > MAX_SPEED_COMMAND) {
    return MAX_SPEED_COMMAND;
  } else if (command < MIN_SPEED_COMMAND) {
    return MIN_SPEED_COMMAND;
  }
  return command;
}

/**
 * @brief Control the speed of a wheel using a PID navigation. The desired
 target speed in rpm is in the pid structure..
 * @param motor The motor structure.
 * @param encoder The encoder structure.
 * @param pid The PID navigation structure.
*/
void wheelSpeedController(Motor* motor, Encoder* encoder, PidController* pid) {
  double errorRpm = motor->wheelSpeed - pid->targetValue;
  double correction = getNewPidValue(pid, errorRpm);
  int newCmd = getClampedCommand(motor->currentCommand - correction);

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