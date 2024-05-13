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

#include "./motorCommands.h"
#include "./taskDcMotor.h"

void initialiseMotor(Motor* motor);
void motorControl(Motor* motor);

void TaskDcMotor(void* pvParameters) {
  // initialise motors
  initialiseMotor(&leftMotor);
  initialiseMotor(&rightMotor);

  // set time delay for ramps
  setParameter(PARAM_MOTOR_RAMP_STEP, 1);  // ms

  while (true) {
    motorControl(&leftMotor);
    motorControl(&rightMotor);
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

void initialiseMotor(Motor* motor) {
  // we will do some PWM on the motor pins
  pinMode(motor->pin1, OUTPUT);
  pinMode(motor->pin2, OUTPUT);
  // initally stop the motor
  analogWrite(motor->pin1, 0);
  analogWrite(motor->pin2, 0);
  setParameter(motor->modeParameter, MOTOR_STOP);
  // initialise motor parameters
  setParameter(motor->speedParameter, 100);
  setParameter(motor->angleParameter, 90);  // degrees
}

void motorControl(Motor* motor) {
  int currentSpeed = getParameter(motor->speedParameter);
  int currentMode = getParameter(motor->modeParameter);

  switch (currentMode) {
    case MOTOR_STOP:
      stopMotor(motor);
      break;
    case MOTOR_CONSTANT_SPEED:
      if (motor->previousMode != currentMode || currentSpeed != motor->speed) {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor constant speed mode");
        }
        speedRamp(motor, currentSpeed, getParameter(PARAM_MOTOR_RAMP_STEP));
      }
      break;
    case MOTOR_MOVE_SECONDS: {
      int delaySeconds = 1;
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.print("Move for ");
        Serial.print(delaySeconds);
        Serial.print(" seconds at speed ");
        Serial.println(currentSpeed);
      }
      moveSeconds(motor, delaySeconds, currentSpeed,
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
        Serial.println(currentSpeed);
      }
      moveDegrees(motor, degrees, currentSpeed);
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
        Serial.println(currentSpeed);
      }
      shortFullSpeed(motor, currentSpeed, 1);
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