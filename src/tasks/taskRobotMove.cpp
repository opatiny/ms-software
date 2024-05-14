/**
 * Thread to control the movement of the robot.
 *
 * Use the serial parameters.
 *
 *   - robot mode: commands A
 *   - robot speed: commands A
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *  - robot angle: commands A (used for rotation modes)
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
#include "./taskDcMotor.h"
#include "./taskRobotMove.h"

void initialiseMotor(Motor* motor, MotorParams* params);
void robotControl(State* state);

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

  // initialise the motors
  initialiseMotor(&state.leftMotor, &leftMotorParams);
  initialiseMotor(&state.rightMotor, &rightMotorParams);

  // set time delay for ramps
  setParameter(PARAM_MOTOR_RAMP_STEP, 1);  // ms

  while (true) {
    robotControl(&state);
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

void robotControl(State* state) {
  int targetSpeed = getParameter(state->robot.speedParameter);
  int currentMode = getParameter(state->robot.modeParameter);

  switch (currentMode) {
    case ROBOT_STOP:
      robotStop();
      break;
    case ROBOT_MOVE:
      robotMove(targetSpeed);
      break;
    case ROBOT_TURN_IN_PLACE:
      robotTurnInPlace(targetSpeed);
      break;
    default:
      Serial.println("Unknown robot movement mode");
      setParameter(state->robot.modeParameter, ROBOT_STOP);
      break;
  }
  state->robot.previousMode = currentMode;
}