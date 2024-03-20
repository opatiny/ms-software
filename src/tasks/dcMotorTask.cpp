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

  while (true) {
    analogWrite(MOTOR_LEFT_PIN1, 0);
    analogWrite(MOTOR_LEFT_PIN2, 0);
    Serial.println("Motor left: OFF");
    vTaskDelay(1000);
    analogWrite(MOTOR_LEFT_PIN1, 255);
    analogWrite(MOTOR_LEFT_PIN2, 0);
    Serial.println("Motor left: ON");
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