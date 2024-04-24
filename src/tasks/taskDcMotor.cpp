/**
 * Thread to control the two DC motors.
 *
 * Use the serial parameters to act on the motors.
 *
 *   - motor mode: commands AC (left) and AD (right)
 *   - motor speed: ommands AA (left) and AB (right)
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forwardß
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

  while (true) {
    int currentSpeed = getParameter(PARAM_MOTOR_LEFT_SPEED);

    switch (getParameter(PARAM_MOTOR_LEFT_MODE)) {
      case MOTOR_STOP:
        stopMotor(&leftMotor);
        break;
      case MOTOR_CONSTANT_SPEED:
        if (currentSpeed != leftMotor.speed) {
          if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
            Serial.println("Motor constant speed mode");
          }
          speedRamp(&leftMotor, currentSpeed, 5);
        }
        break;
      case MOTOR_MOVE_SECONDS: {
        int delaySeconds = 5;
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Start motor move seconds mode");
          Serial.print("Speed: ");
          Serial.println(currentSpeed);
          Serial.print("Seconds: ");
          Serial.println(delaySeconds);
        }
        moveSeconds(&leftMotor, delaySeconds, currentSpeed, 1);
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("End motor move seconds mode");
        }
        break;
      }
      case MOTOR_MOVE_DEGREES:
        Serial.println("Motor move degrees mode");
        break;
      case MOTOR_SHORT: {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor short pulse mode (only for debug)");
          Serial.print("Speed: ");
          Serial.println(currentSpeed);
        }
        shortFullSpeed(&leftMotor, currentSpeed);
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;
      }
      default:
        Serial.println("Unknown motor mode");
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;
    }
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
