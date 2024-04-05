/**
 * Thread to handle the control of the two DC motors.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskDcMotor.h"

void rampUpDown(int pin1, int pin2, int speed);
void shortFullSpeed(int pin1, int pin2, int speed);

void TaskDcMotor(void* pvParameters) {
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);

  // initally stop the motor
  analogWrite(MOTOR_LEFT_PIN1, 0);
  analogWrite(MOTOR_LEFT_PIN2, 0);

  while (true) {
    int leftSpeed = getParameter(PARAM_MOTOR_LEFT_SPEED);

    switch (getParameter(PARAM_MOTOR_LEFT_MODE)) {
      case MOTOR_STOP:
        analogWrite(MOTOR_LEFT_PIN1, 0);
        analogWrite(MOTOR_LEFT_PIN2, 0);
        break;
      case MOTOR_FORWARD:
        analogWrite(MOTOR_LEFT_PIN1, leftSpeed);
        analogWrite(MOTOR_LEFT_PIN2, 0);
        break;
      case MOTOR_BACKWARD:
        analogWrite(MOTOR_LEFT_PIN1, 0);
        analogWrite(MOTOR_LEFT_PIN2, leftSpeed);
        break;
      case MOTOR_RAMP:
        rampUpDown(MOTOR_LEFT_PIN1, MOTOR_LEFT_PIN2, leftSpeed);
        break;
      case MOTOR_SHORT:
        shortFullSpeed(MOTOR_LEFT_PIN1, MOTOR_LEFT_PIN2, leftSpeed);
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;

      default:
        Serial.println("Unknown motor mode");
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;
    }
    vTaskDelay(10);
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

void rampUpDown(int pin1, int pin2, int speed) {
  for (int i = 0; i < 255; i++) {
    analogWrite(pin1, i);
    vTaskDelay(10);
  }
  for (int i = 255; i > 0; i--) {
    analogWrite(pin1, i);
    vTaskDelay(10);
  }
  for (int i = 0; i < 255; i++) {
    analogWrite(pin2, i);
    vTaskDelay(10);
  }
  for (int i = 255; i > 0; i--) {
    analogWrite(pin2, i);
    vTaskDelay(10);
  }
}

void shortFullSpeed(int pin1, int pin2, int speed) {
  analogWrite(pin1, speed);
  analogWrite(pin2, 0);
  vTaskDelay(1000);
}