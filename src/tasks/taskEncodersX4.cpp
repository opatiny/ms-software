/**
 * Thread to handle the control of the encoders of the motors.
 * This thread uses X4 encoding.
 * This should give us 360 counts per revolution of the wheel.
 *
 * Debug: U5
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "../pinMapping.h"
#include "../state.h"

#include "taskEncoders.h"

/**
 * Delay between each encoder reading.
 */
#define DELAY 50

void counterRoutine(EncoderCounter* counter,
                    int interruptPin,
                    int directionPin,
                    int increment);

void leftCounterPin1();
void leftCounterPin2();
void rightCounterPin1();
void rightCounterPin2();

void TaskEncodersX4(void* pvParameters) {
  // left encoder
  pinMode(LEFT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_PIN2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN1), leftCounterPin1,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN2), leftCounterPin2,
                  CHANGE);

  // right encoder
  pinMode(RIGHT_ENCODER_PIN1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN1), rightCounterPin1,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN2), rightCounterPin2,
                  CHANGE);

  while (true) {
    vTaskDelay(DELAY);
    if (getParameter(PARAM_DEBUG) == DEBUG_ENCODERS) {
      int time = millis();
      Serial.print(time);
      Serial.print(", \t");
      Serial.print(robot.leftEncoder.counts);
      Serial.print(", \t");
      Serial.print(robot.rightEncoder.counts);
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

void leftCounterPin1() {
  counterRoutine(&(robot.leftEncoder.counts), LEFT_ENCODER_PIN1,
                 LEFT_ENCODER_PIN2, 1);
}

void leftCounterPin2() {
  counterRoutine(&(robot.leftEncoder.counts), LEFT_ENCODER_PIN2,
                 LEFT_ENCODER_PIN1, -1);
}

void rightCounterPin1() {
  counterRoutine(&(robot.leftEncoder.counts), RIGHT_ENCODER_PIN1,
                 RIGHT_ENCODER_PIN2, 1);
}

void rightCounterPin2() {
  counterRoutine(&(robot.leftEncoder.counts), RIGHT_ENCODER_PIN2,
                 RIGHT_ENCODER_PIN1, -1);
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
void counterRoutine(EncoderCounter* counter,
                    int interruptPin,
                    int directionPin,
                    int increment) {
  int interruptValue = digitalRead(interruptPin);
  int directionValue = digitalRead(directionPin);

  EncoderCounter newValue = *counter;
  if (interruptValue == HIGH) {
    if (directionValue == HIGH) {
      newValue = newValue - increment;
    } else {
      newValue = newValue + increment;
    }
  } else {
    if (directionValue == HIGH) {
      newValue = newValue + increment;
    } else {
      newValue = newValue - increment;
    }
  }
  *counter = newValue;
}

void initialiseEncoderCounter(Encoder* encoder) {
  pinMode(encoder->pin1, INPUT_PULLUP);
  pinMode(encoder->pin2, INPUT_PULLUP);
  encoder->counts = 0;
}