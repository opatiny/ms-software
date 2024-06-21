/**
 * Thread to control the movement of the controller.
 *
 * Use the serial parameters.
 *
 *   - robot mode: AO
 *   - robot speed in rpm: AN
 *   - robot command: AM
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *  - robot angle: AP (used for rotation modes)
 *  - robot distance: A...
 *  - robot radius: commands A... (used for arc mode)
 *
 * Debug: U7
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
  PidSerialParameters serialParamsPid = {
    kp : PARAM_CONTROLLER_KP,
    ki : PARAM_CONTROLLER_KI,
    kd : PARAM_CONTROLLER_KD,
  };

  ControllerParams robotParams = {
    commandParameter : PARAM_ROBOT_COMMAND,
    speedParameter : PARAM_ROBOT_WHEELS_SPEED,
    modeParameter : PARAM_ROBOT_MODE,
    angleParameter : PARAM_ROBOT_ANGLE_CMD,
    pidParams : serialParamsPid,
  };

  initialiseController(&robot.navigation, &robotParams);

  // initialise the motors
  initialiseMotor(&robot.leftMotor, &leftMotorParams);
  initialiseMotor(&robot.rightMotor, &rightMotorParams);

  while (true) {
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
  int targetSpeed = getParameter(robot->navigation.speedParameter);
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

  if (robot->navigation.previousMode != currentMode) {
    // clear the controllers when changing back to a mode that uses them
    if (currentMode == ROBOT_MOVE_STRAIGHT ||
        currentMode == ROBOT_STOP_OBSTACLE) {
      robot->navigation.wheelsSpeedController.clearControllers = 1;
    }
    if (getParameter(PARAM_DEBUG) == DEBUG_ROBOT_CONTROL) {
      Serial.print("New robot mode: ");
      Serial.println(currentMode);
    }
  }

  switch (currentMode) {
    case ROBOT_STOP:
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
    case ROBOT_MOVE_STRAIGHT:
      robotMoveStraight(robot, targetSpeed);

      break;
    default:
      Serial.println("Unknown robot movement mode");
      setParameter(robot->navigation.modeParameter, ROBOT_STOP);
      break;
  }

  robot->navigation.previousMode = currentMode;
}
