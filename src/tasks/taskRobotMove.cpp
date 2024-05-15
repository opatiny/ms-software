/**
 * Thread to control the movement of the controller.
 *
 * Use the serial parameters.
 *
 *   - robot mode: commands AO
 *   - robot speed: commands AN
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *  - robot angle: commands AP (used for rotation modes)
 *  - robot distance: commands A
 *  - robot radius: commands A (used for arc mode)
 *
 * Debug: U10
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "../pinMapping.h"
#include "../state.h"
#include "./motorCommands.h"
#include "./robotCommands.h"
#include "./taskButton.h"
#include "./taskDcMotor.h"
#include "./taskRobotMove.h"

void initialiseMotor(Motor* motor, MotorParams* params);
void robotControl(Robot* robot);

void TaskRobotMove(void* pvParameters) {
  // define parameters of the motors
  MotorParams leftMotorParams = {
    speedParameter : PARAM_MOTOR_LEFT_SPEED_CMD,
    modeParameter : PARAM_MOTOR_LEFT_MODE,
    angleParameter : PARAM_MOTOR_LEFT_ANGLE_CMD,
    pin1 : MOTOR_LEFT_PIN1,
    pin2 : MOTOR_LEFT_PIN2
  };

  MotorParams rightMotorParams = {
    speedParameter : PARAM_MOTOR_RIGHT_SPEED_CMD,
    modeParameter : PARAM_MOTOR_RIGHT_MODE,
    angleParameter : PARAM_MOTOR_RIGHT_ANGLE_CMD,
    pin1 : MOTOR_RIGHT_PIN1,
    pin2 : MOTOR_RIGHT_PIN2
  };

  ControllerParams robotParams = {
    speedParameter : PARAM_ROBOT_SPEED_CMD,
    modeParameter : PARAM_ROBOT_MODE,
    angleParameter : PARAM_ROBOT_ANGLE_CMD,
    rampStepParameter : PARAM_MOTOR_RAMP_STEP
  };

  initialiseController(&robot.controller, &robotParams);

  // initialise the motors
  initialiseMotor(&robot.leftMotor, &leftMotorParams);
  initialiseMotor(&robot.rightMotor, &rightMotorParams);

  while (true) {
    robotControl(&robot);
    vTaskDelay(1000);
  }
}

void taskRobotMove() {
  xTaskCreatePinnedToCore(TaskRobotMove, "TaskRobotMove",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void robotControl(Robot* robot) {
  int targetSpeed = getParameter(robot->controller.speedParameter);
  int currentMode = getParameter(robot->controller.modeParameter);

  if (buttonFlags.robotMode) {
    if (currentMode == ROBOT_MOVE) {
      currentMode = ROBOT_STOP;
    } else {
      currentMode = ROBOT_MOVE;
    }
    Serial.println("Button pressed!");
    setParameter(robot->controller.modeParameter, currentMode);
    buttonFlags.robotMode = false;
  }

  if (robot->controller.previousMode != currentMode) {
    Serial.print("New robot mode: ");
    Serial.println(currentMode);
    switch (currentMode) {
      case ROBOT_STOP:
        robotStop(robot);
        break;
      case ROBOT_MOVE:
        robotMove(robot, targetSpeed);
        break;
      case ROBOT_TURN_IN_PLACE:
        robotTurnInPlace(robot, targetSpeed);
        break;
      case ROBOT_STOP_OBSTACLE:
        stopWhenObstacle(robot, targetSpeed, 100);
        break;
      default:
        Serial.println("Unknown robot movement mode");
        setParameter(robot->controller.modeParameter, ROBOT_STOP);
        break;
    }
  }
  robot->controller.previousMode = currentMode;
}