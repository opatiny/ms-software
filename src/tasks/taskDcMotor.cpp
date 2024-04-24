/**
 * Thread to handle the control of the two DC motors.
 */

#include <globalConfig.h>
#include <utilities/params.h>

#include "./motorCommands.h"
#include "./taskDcMotor.h"

#define DEFAULT_RAMP_DELAY 1

enum Direction { FORWARD, BACKWARD };

void stopMotor(Motor* motor);
void rampDown(Motor* motor,
              Direction direction,
              int finalSpeed,
              int rampDelay = DEFAULT_RAMP_DELAY);
void rampUp(Motor* motor,
            Direction direction,
            int finalSpeed,
            int rampDelay = DEFAULT);
void speedRamp(Motor* motor, int finalSpeed, int rampDelay);

void TaskDcMotor(void* pvParameters) {
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);

  // initally stop the motor
  analogWrite(MOTOR_LEFT_PIN1, 0);
  analogWrite(MOTOR_LEFT_PIN2, 0);

  while (true) {
    int currentSpeed = getParameter(PARAM_MOTOR_LEFT_SPEED);

    Serial.print("Motor left current speed: ");
    Serial.println(leftMotor.speed);
    Serial.print("Motor left goal speed: ");
    Serial.println(currentSpeed);
    Serial.print("Motor left mode: ");
    Serial.println(getParameter(PARAM_MOTOR_LEFT_MODE));

    // change motor mode with commands AC (left) and AD (right)
    switch (getParameter(PARAM_MOTOR_LEFT_MODE)) {
      case MOTOR_STOP:
        stopMotor(&leftMotor);
        break;
      case MOTOR_CONSTANT_SPEED:
        Serial.println("Motor constant speed");
        if (currentSpeed != leftMotor.speed) {
          speedRamp(&leftMotor, currentSpeed);
        }
        break;
      case MOTOR_MOVE_DEGREES:
        break;

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

void stopMotor(Motor* motor) {
  if (motor->speed > 0) {
    rampDown(motor, FORWARD, 0);
  } else if (motor->speed < 0) {
    rampDown(motor, BACKWARD, 0);
  }
  motor->speed = 0;
  setParameter(motor->speedParameter, 0);
}

void speedRamp(Motor* motor,
               int finalSpeed,
               int rampDelay = DEFAULT_RAMP_DELAY) {
  int initialSpeed = motor->speed;
  if (initialSpeed > finalSpeed) {
    Serial.println("i > f");
    if (initialSpeed > 0) {
      if (finalSpeed > 0) {
        rampDown(motor, FORWARD, finalSpeed, rampDelay);
      } else {
        rampDown(motor, FORWARD, 0, rampDelay);
        rampUp(motor, BACKWARD, -finalSpeed, rampDelay);
      }
    } else {
      Serial.println("negative or zero speed");
      rampUp(motor, BACKWARD, -finalSpeed, rampDelay);
    }
  } else {
    Serial.println("i < f");
    if (initialSpeed > 0) {
      if (finalSpeed > 0) {
        rampUp(motor, FORWARD, finalSpeed, rampDelay);
      } else {
        rampDown(motor, BACKWARD, 0, rampDelay);
        rampUp(motor, FORWARD, finalSpeed, rampDelay);
      }
    } else {
      Serial.println("negative or zero speed");
      Serial.println("initialSpeed: ");
      Serial.println(initialSpeed);
      rampDown(motor, BACKWARD, -finalSpeed, rampDelay);
    }
  }
  motor->speed = finalSpeed;
  setParameter(motor->speedParameter, finalSpeed);
}

void rampUp(Motor* motor,
            Direction direction,
            int finalSpeed,
            int rampDelay = DEFAULT_RAMP_DELAY) {
  int initialSpeed = motor->speed;
  if (initialSpeed < 0) {
    initialSpeed = -initialSpeed;
  }
  switch (direction) {
    case FORWARD:
      for (int speed = initialSpeed; speed < finalSpeed; speed++) {
        analogWrite(motor->pin1, speed);
        analogWrite(motor->pin2, 0);
        vTaskDelay(rampDelay);
      }
      break;
    case BACKWARD:
      for (int speed = initialSpeed; speed < finalSpeed; speed++) {
        analogWrite(motor->pin1, 0);
        analogWrite(motor->pin2, speed);
        vTaskDelay(rampDelay);
      }
      break;
  }
}

void rampDown(Motor* motor,
              Direction direction,
              int finalSpeed,
              int rampDelay = DEFAULT_RAMP_DELAY) {
  int initialSpeed = motor->speed;
  if (initialSpeed < 0) {
    initialSpeed = -initialSpeed;
  }
  switch (direction) {
    case FORWARD:
      for (int speed = initialSpeed; speed > finalSpeed; speed--) {
        analogWrite(motor->pin1, speed);
        analogWrite(motor->pin2, 0);
        vTaskDelay(rampDelay);
      }
      break;
    case BACKWARD:
      for (int speed = initialSpeed; speed > finalSpeed; speed--) {
        analogWrite(motor->pin1, 0);
        analogWrite(motor->pin2, speed);
        vTaskDelay(rampDelay);
      }
      break;
  }
}