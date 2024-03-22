/**
 * Thread to handle the control of the two DC motors.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#define MOTOR_LEFT_PIN1 D9
#define MOTOR_LEFT_PIN2 D10

void TaskDcMotor(void* pvParameters) {
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);

  // initally stop the motor
  analogWrite(MOTOR_LEFT_PIN1, 0);
  analogWrite(MOTOR_LEFT_PIN2, 0);

  while (true) {
    for (int i = 0; i < 255; i++) {
      analogWrite(MOTOR_LEFT_PIN1, i);
      vTaskDelay(10);
    }
    for (int i = 255; i > 0; i--) {
      analogWrite(MOTOR_LEFT_PIN1, i);
      vTaskDelay(10);
    }
    for (int i = 0; i < 255; i++) {
      analogWrite(MOTOR_LEFT_PIN2, i);
      vTaskDelay(10);
    }
    for (int i = 255; i > 0; i--) {
      analogWrite(MOTOR_LEFT_PIN2, i);
      vTaskDelay(10);
    }
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