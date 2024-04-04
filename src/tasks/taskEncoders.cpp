/**
 * Thread to handle the control of the encoders of the motors.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskEncoders.h"

void counterRoutine();

int counterLeftPin1 = 0;
int counterLeftPin2 = 0;

int counter = 0;
void TaskEncoders(void* pvParameters) {
  pinMode(LEFT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN1), counterRoutine,
                  CHANGE);

  while (true) {
    vTaskDelay(1000);
    if (parameters[PARAM_DEBUG] == DEBUG_ENCODERS)
      Serial.println(counter);
  }
}

void taskEncoders() {
  xTaskCreatePinnedToCore(TaskEncoders, "TaskEncoders",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void counterRoutine() {
  counter++;
}