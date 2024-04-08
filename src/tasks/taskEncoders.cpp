/**
 * Thread to handle the control of the encoders of the motors.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskEncoders.h"

void counterRoutine(int parameterPin, int directionPin);

void leftCounter();
void rightCounter();

void TaskEncoders(void* pvParameters) {
  pinMode(LEFT_ENCODER_COUNTER_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_DIRECTION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_COUNTER_PIN), leftCounter,
                  RISING);

  while (true) {
    vTaskDelay(1000);
    if (parameters[PARAM_DEBUG] == DEBUG_ENCODERS)
      Serial.println(getParameter(PARAM_ENCODER_LEFT));
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

void leftCounter() {
  counterRoutine(PARAM_ENCODER_LEFT, LEFT_ENCODER_DIRECTION_PIN);
}

void rightCounter() {
  counterRoutine(PARAM_ENCODER_RIGHT, RIGHT_ENCODER_DIRECTION_PIN);
}

void counterRoutine(int parameterPin, int directionPin) {
  int newValue = getParameter(parameterPin);

  Serial.print("Direction pin: ");
  Serial.println(digitalRead(directionPin));

  if (digitalRead(directionPin) == HIGH) {
    newValue--;
  } else {
    newValue++;
  }
  setParameter(parameterPin, newValue);
}
