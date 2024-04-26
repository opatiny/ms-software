/**
 * Thread to handle the control of the encoders of the motors.
 * This thread uses X4 encoding.
 * This should give us 360 counts per revolution of the wheel.
 *
 * Debug: U5
 * Log data for matlab: U8
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskEncoders.h"

#define DELAY 5

void counterRoutine(int parameterPin,
                    int interruptPin,
                    int directionPin,
                    int increment);
void leftCounterPin1();
void leftCounterPin2();

void TaskEncodersX4(void* pvParameters) {
  pinMode(LEFT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN1), leftCounterPin1,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN2), leftCounterPin2,
                  CHANGE);

  int encoderTime = 0;
  while (true) {
    vTaskDelay(DELAY);
    if (getParameter(PARAM_DEBUG) == DEBUG_ENCODERS) {
      Serial.println(getParameter(PARAM_ENCODER_LEFT));
    } else if (getParameter(PARAM_DEBUG) == DEBUG_ENCODERS_LOG_DATA) {
      int leftEncoder = getParameter(PARAM_ENCODER_LEFT);
      int rightEncoder = getParameter(PARAM_ENCODER_RIGHT);
      Serial.print(encoderTime);
      Serial.print(", \t");
      encoderTime += DELAY;
      Serial.print(leftEncoder);
      Serial.print(", \t");
      Serial.print(rightEncoder);
      Serial.print(", \t");
      Serial.print(getParameter(PARAM_MOTOR_LEFT_MODE));
      Serial.print(", \t");
      Serial.println(getParameter(PARAM_MOTOR_RIGHT_MODE));
    }
  }
}

void taskEncodersX4() {
  xTaskCreatePinnedToCore(TaskEncodersX4, "TaskEncodersX4",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}
uint32_t timeLeft = 0;

void leftCounterPin1() {
  counterRoutine(PARAM_ENCODER_LEFT, LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2, 1);
}

void leftCounterPin2() {
  counterRoutine(PARAM_ENCODER_LEFT, LEFT_ENCODER_PIN2, LEFT_ENCODER_PIN1, -1);
}

/**
 * Interrupt routine to count the encoder pulses.
 *
 * X4 encoding: we read changes on both pins (two interrupts) per encoder.
 *
 *  @param parameter - The parameter containing the encoder counter.
 *  @param interruptPin - The pin used for counting the encoder pulses.
 *  @param directionPin - The pin used to determine the direction of the
 * rotation.
 */
void counterRoutine(int parameter,
                    int interruptPin,
                    int directionPin,
                    int increment) {
  int interruptValue = digitalRead(interruptPin);
  int directionValue = digitalRead(directionPin);

  int newValue = getParameter(parameter);
  if (interruptValue == HIGH) {
    if (directionValue == HIGH) {
      newValue = newValue + increment;
    } else {
      newValue = newValue - increment;
    }
  } else {
    if (directionValue == HIGH) {
      newValue = newValue - increment;
    } else {
      newValue = newValue + increment;
    }
  }
  setParameter(parameter, newValue);
}