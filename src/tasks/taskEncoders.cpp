/**
 * Thread to handle the control of the encoders of the motors.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskEncoders.h"

void counterRoutine(int parameterPin, int interruptPin, int directionPin);
void leftCounterPin1();
void leftCounterPin2();

void TaskEncoders(void* pvParameters) {
  pinMode(LEFT_ENCODER_PIN2, INPUT);
  pinMode(LEFT_ENCODER_PIN1, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN2), leftCounterPin2,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN1), leftCounterPin1,
                  CHANGE);

  while (true) {
    vTaskDelay(200);
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

void leftCounterPin1() {
  counterRoutine(PARAM_ENCODER_LEFT, LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2);
}

void leftCounterPin2() {
  counterRoutine(PARAM_ENCODER_LEFT, LEFT_ENCODER_PIN2, LEFT_ENCODER_PIN1);
}

/**
 * Interrupt routine to count the encoder pulses.
 * X4 encoding: we read changes on both pins (two interrupts).
 */
void counterRoutine(int parameterPin, int interruptPin, int directionPin) {
  int interruptValue = digitalRead(interruptPin);
  int directionValue = digitalRead(directionPin);

  Serial.print("interruptValue: ");
  Serial.println(interruptValue);
  Serial.print("directionValue: ");
  Serial.println(directionValue);

  int newValue = getParameter(parameterPin);
  if (interruptValue == HIGH) {
    if (directionValue == HIGH) {
      newValue++;
    } else {
      newValue--;
    }
  } else {
    if (directionValue == HIGH) {
      newValue--;
    } else {
      newValue++;
    }
  }
}