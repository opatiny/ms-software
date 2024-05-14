/**
 * Thread to control the two DC motors.
 *
 * Use the serial parameters to act on the motors.
 *
 *   - motor mode: commands AC (left) and AD (right)
 *   - motor speed: commands AA (left) and AB (right)
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *  - motor angle: commands AJ (left) and AK (right)
 *
 * Debug: U7
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "../pinMapping.h"
#include "../state.h"
#include "./motorCommands.h"
#include "./taskDcMotor.h"

void initialiseMotor(Motor* motor, MotorParams* params);
void motorControl(Motor* motor);

void TaskDcMotor(void* pvParameters) {
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
    motorControl(&state.leftMotor);
    motorControl(&state.rightMotor);
    vTaskDelay(1000);
  }
}

void taskDcMotor() {
  xTaskCreatePinnedToCore(TaskDcMotor, "TaskDcMotor",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void motorControl(Motor* motor) {
  int targetSpeed = getParameter(motor->speedParameter);
  int currentMode = getParameter(motor->modeParameter);

  switch (currentMode) {
    case MOTOR_STOP:
      stopMotor(motor);
      break;
    case MOTOR_CONSTANT_SPEED:
      if (motor->previousMode != currentMode ||
          targetSpeed != motor->currentSpeed) {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor constant speed mode");
        }
        speedRamp(motor, targetSpeed, getParameter(PARAM_MOTOR_RAMP_STEP));
      }
      break;
    case MOTOR_MOVE_SECONDS: {
      int delaySeconds = 1;
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.print("Move for ");
        Serial.print(delaySeconds);
        Serial.print(" seconds at speed ");
        Serial.println(targetSpeed);
      }
      moveSeconds(motor, delaySeconds, targetSpeed,
                  getParameter(PARAM_MOTOR_RAMP_STEP));
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.println("End of movement");
      }
      break;
    }
    case MOTOR_MOVE_DEGREES: {
      int degrees = getParameter(motor->angleParameter);
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.print("Move for ");
        Serial.print(degrees);
        Serial.print(" degrees at speed ");
        Serial.println(targetSpeed);
      }
      moveDegrees(motor, degrees, targetSpeed);
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
      }
      break;
    }
    case MOTOR_SHORT: {
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        int delay = 1;
        Serial.println("Short pulse mode (only for debug)");
        Serial.print("Motor will spin for ");
        Serial.print(delay);
        Serial.print(" seconds at speed ");
        Serial.println(targetSpeed);
      }
      shortFullSpeed(motor, targetSpeed, 1);
      setParameter(motor->modeParameter, MOTOR_STOP);
      break;
    }
    default:
      Serial.println("Unknown motor mode");
      setParameter(motor->modeParameter, MOTOR_STOP);
      break;
  }
  motor->previousMode = currentMode;
}