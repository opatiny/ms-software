/**
 * Measure motor rotation by using X1 encoding.
 * This gives us 90 counts per revolution of the wheel.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "../pinMapping.h"
#include "motorCommands.h"
#include "taskEncoders.h"

// Pointers to the encoders counters of the motors.
Encoder* leftEncoderPt = &(leftMotor.encoderCounts);
Encoder* rightEncoderPt = &(rightMotor.encoderCounts);

void counterRoutine(Encoder* encoderCounter, int directionPin);

void leftCounter();
void rightCounter();

void TaskEncodersX1(void* pvParameters) {
  pinMode(LEFT_ENCODER_PIN2, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN2), leftCounter,
                  RISING);

  while (true) {
    vTaskDelay(200);
    if (parameters[PARAM_DEBUG] == DEBUG_ENCODERS)
      Serial.println(*leftEncoderPt);
  }
}

void taskEncodersX1() {
  xTaskCreatePinnedToCore(TaskEncodersX1, "TaskEncodersX1",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void leftCounter() {
  counterRoutine(leftEncoderPt, LEFT_ENCODER_PIN1);
}

void rightCounter() {
  counterRoutine(leftEncoderPt, LEFT_ENCODER_PIN1);
}

void counterRoutine(Encoder* encoderCounter, int directionPin) {
  int newValue = *encoderCounter;

  if (digitalRead(directionPin) == HIGH) {
    newValue--;
  } else {
    newValue++;
  }
  *encoderCounter = newValue;
}