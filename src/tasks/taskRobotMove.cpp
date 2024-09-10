/**
 * Thread to control the movement of the controller.
 *
 * Use the serial parameters.
 *
 *   - robot mode: T
 *   - wheels speed in rpm: Q
 *   - robot command: P
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *   - for control with regulation:
 *     - linear speed in mm/s: R
 *     - angular speed in deg/s: S
 * Debug: A7
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include <motorCommands.h>
#include <pinMapping.h>
#include <robotCommands.h>
#include <robotModes.h>
#include <state.h>
#include "taskButton.h"
#include "taskRobotMove.h"

void initialiseMotor(Motor* motor, MotorParams* params);
void robotControl(Robot* robot);

void TaskRobotMove(void* pvParameters) {
  // define parameters of the motors
  MotorParams leftMotorParams = {
    commandParameter : PARAM_MOTOR_LEFT_COMMAND,
    modeParameter : PARAM_MOTOR_LEFT_MODE,
    accDurationParameter : PARAM_MOTOR_ACC_DURATION,
    pin1 : MOTOR_LEFT_PIN1,
    pin2 : MOTOR_LEFT_PIN2
  };

  MotorParams rightMotorParams = {
    commandParameter : PARAM_MOTOR_RIGHT_COMMAND,
    modeParameter : PARAM_MOTOR_RIGHT_MODE,
    accDurationParameter : PARAM_MOTOR_ACC_DURATION,
    pin1 : MOTOR_RIGHT_PIN1,
    pin2 : MOTOR_RIGHT_PIN2
  };

  // pid control to ensure both wheels go at same speed
  PidInitParameters wheelsPid = {
    serialParams : {
      kp : PARAM_WHEEL_KP,
      ki : PARAM_WHEEL_KI,
      kd : PARAM_WHEEL_KD,
    },
    factor : 1000,
  };

  PidInitParameters linearPid = {
    serialParams : {
      kp : PARAM_LINEAR_KP,
      ki : PARAM_LINEAR_KI,
      kd : PARAM_LINEAR_KD,
    },
    factor : 10,
  };

  PidInitParameters angularPid = {
    serialParams : {
      kp : PARAM_ANGULAR_KP,
      ki : PARAM_ANGULAR_KI,
      kd : PARAM_ANGULAR_KD,
    },
    factor : 100,
  };

  ControllerParams robotParams = {
    wheelsPid : wheelsPid,
    linearPid : linearPid,
    angularPid : angularPid,
  };
  Serial.println("Initialising navigation");
  initialiseNavigation(&robot.navigation, &robotParams);

  Serial.println("Initialising motors");
  // initialise the motors
  initialiseMotor(&robot.leftMotor, &leftMotorParams);
  initialiseMotor(&robot.rightMotor, &rightMotorParams);

  while (true) {
    debugProcess("TaskRobotMove ");
    robotControl(&robot);
    vTaskDelay(1);  // smallest delay possible -> there should be no other
                    // delays in this task!!
  }
}

void taskRobotMove() {
  xTaskCreatePinnedToCore(TaskRobotMove, "TaskRobotMove", 4096, NULL, 2, NULL,
                          0);  // attached on core 0!!
}

void robotControl(Robot* robot) {
  int targetCommand = getParameter(robot->navigation.commandParameter);
  int targetSpeed = getParameter(robot->navigation.wheelSpeedParameter);
  int currentMode = getParameter(robot->navigation.modeParameter);

  if (buttonFlags.robotMode) {
    if (currentMode != ROBOT_STOP) {
      currentMode = ROBOT_STOP;
    } else {
      currentMode = ROBOT_STOP_OBSTACLE;
    }
    if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
      Serial.println("Button pressed!");
    }
    setParameter(robot->navigation.modeParameter, currentMode);
    buttonFlags.robotMode = false;
  }
  // Serial.println("Button flags done");
  if (robot->navigation.previousMode != currentMode) {
    // clear the controllers when changing back to a mode that uses them
    if (currentMode == ROBOT_WHEEL_SPEED_CONTROL ||
        currentMode == ROBOT_STOP_OBSTACLE) {
      robot->navigation.wheelsSpeedController.left.clearController = 1;
      robot->navigation.wheelsSpeedController.right.clearController = 1;
    }
    if (currentMode == ROBOT_SPEED_CONTROL) {
      robot->navigation.robotSpeedController.linear.clearController = 1;
      robot->navigation.robotSpeedController.angular.clearController = 1;
    }
    if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
      Serial.print("New robot mode: ");
      Serial.println(currentMode);
    }
    robot->navigation.previousMode = currentMode;
  }

  // Serial.println("Clear controller flags done");
  // Serial.print("Current mode: ");
  // Serial.println(currentMode);
  switch (currentMode) {
    case ROBOT_STOP:
      // Serial.println("Robot stop");
      stopMotors(robot);
      break;
    case ROBOT_MOVE_SAME_COMMAND:
      robotMoveSameCommand(robot, targetCommand);
      break;
    case ROBOT_MOVE:
      robotMove(robot, targetSpeed);
      break;
    case ROBOT_TURN_IN_PLACE:
      robotTurnInPlace(robot, targetSpeed);
      break;
    case ROBOT_STOP_OBSTACLE: {
      int distance = getParameter(PARAM_OBSTACLE_DISTANCE);
      stopWhenObstacle(robot, targetSpeed, distance);
      break;
    }
    case ROBOT_EACH_WHEEL:
      motorControl(&robot->leftMotor, &robot->leftEncoder);
      motorControl(&robot->rightMotor, &robot->rightEncoder);
      break;
    case ROBOT_WHEEL_SPEED_CONTROL:
      wheelSpeedControl(robot, targetSpeed);
      break;
    case ROBOT_SPEED_CONTROL: {
      double linSpeed = getParameter(PARAM_ROBOT_SPEED_LIN) / 1000.0;  // m/s
      double angSpeed =
          getParameter(PARAM_ROBOT_SPEED_ANG) * DEG_TO_RAD;  // rad/s

      robotSpeedControl(robot, linSpeed, angSpeed);
      break;
    }
    default:
      Serial.println("Unknown robot movement mode");
      setParameter(robot->navigation.modeParameter, ROBOT_STOP);
      break;
  }

  robot->navigation.previousMode = currentMode;
}
