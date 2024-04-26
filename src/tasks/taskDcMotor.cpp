/**
 * Thread to control the two DC motors.
 *
 * Use the serial parameters to act on the motors.
 *
 *   - motor mode: commands AC (left) and AD (right)
 *   - motor speed: ommands AA (left) and AB (right)
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *
 * Debug for this thread: U7
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./motorCommands.h"
#include "./taskDcMotor.h"

void TaskDcMotor(void* pvParameters) {
  // we will do some PWM on the motor pins
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);

  // initally stop the motor
  analogWrite(MOTOR_LEFT_PIN1, 0);
  analogWrite(MOTOR_LEFT_PIN2, 0);
  setParameter(PARAM_MOTOR_LEFT_SPEED, 0);
  setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
  setParameter(PARAM_MOTOR_RIGHT_SPEED, 0);
  setParameter(PARAM_MOTOR_RIGHT_MODE, MOTOR_STOP);

  setParameter(PARAM_MOTOR_DELAY, 10);  // ms

  int previousMode = MOTOR_STOP;

  while (true) {
    int currentSpeed = getParameter(PARAM_MOTOR_LEFT_SPEED);
    int currentMode = getParameter(PARAM_MOTOR_LEFT_MODE);

    switch (currentMode) {
      case MOTOR_STOP:
        stopMotor(&leftMotor);
        break;
      case MOTOR_CONSTANT_SPEED:
        if (previousMode != currentMode || currentSpeed != leftMotor.speed) {
          if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
            Serial.println("Motor constant speed mode");
          }
          speedRamp(&leftMotor, currentSpeed, getParameter(PARAM_MOTOR_DELAY));
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
        moveSeconds(&leftMotor, delaySeconds, currentSpeed, 10);
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("End of movement");
        }
        break;
      }
      case MOTOR_MOVE_DEGREES:
        Serial.println("Motor move degrees mode");
        break;
      case MOTOR_SHORT: {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          int delay = 1;
          Serial.println("Short pulse mode (only for debug)");
          Serial.print("Motor will spin for ");
          Serial.print(delay);
          Serial.print(" seconds at speed ");
          Serial.println(currentSpeed);
        }
        shortFullSpeed(&leftMotor, currentSpeed, 5);
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;
      }
      default:
        Serial.println("Unknown motor mode");
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;
    }
    previousMode = currentMode;
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
