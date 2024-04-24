/**
 * Thread to control the two DC motors.
 *
 * Use the serial parameters to act on the motors.
 *
 *   - motor mode: commands AC (left) and AD (right)
 *   - motor speed: ommands AA (left) and AB (right)
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forwardß
 *     - negative values means backward
 *
 * Debug for this thread: U7
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
            int rampDelay = DEFAULT_RAMP_DELAY);
void speedRamp(Motor* motor,
               int finalSpeed,
               int rampDelay = DEFAULT_RAMP_DELAY);
void shortFullSpeed(Motor* motor, int speed);

void TaskDcMotor(void* pvParameters) {
  pinMode(MOTOR_LEFT_PIN1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN2, OUTPUT);

  // initally stop the motor
  analogWrite(MOTOR_LEFT_PIN1, 0);
  analogWrite(MOTOR_LEFT_PIN2, 0);

  while (true) {
    int currentSpeed = getParameter(PARAM_MOTOR_LEFT_SPEED);

    switch (getParameter(PARAM_MOTOR_LEFT_MODE)) {
      case MOTOR_STOP:
        stopMotor(&leftMotor);
        break;
      case MOTOR_CONSTANT_SPEED:
        if (currentSpeed != leftMotor.speed) {
          if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
            Serial.println("Motor constant speed mode");
          }
          speedRamp(&leftMotor, currentSpeed, 5);
        }
        break;
      case MOTOR_MOVE_DEGREES:
        Serial.println("Motor move degrees mode");
        break;
      case MOTOR_SHORT: {
        int speed = getParameter(PARAM_MOTOR_LEFT_SPEED);
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor short pulse mode (only for debug)");
          Serial.print("Speed: ");
          Serial.println(speed);
        }
        shortFullSpeed(&leftMotor, speed);
        setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
        break;
      }
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

/**
 * Stop the given motor using a ramp down.
 * @param motor - Struct of the motor to stop
 */
void stopMotor(Motor* motor) {
  if (motor->speed > 0) {
    rampDown(motor, FORWARD, 0);
  } else if (motor->speed < 0) {
    rampDown(motor, BACKWARD, 0);
  }
  motor->speed = 0;
}

/**
 * Change the speed of the motor using a ramp up or down. The speed is an
 * integer in the range [-255,255]. Negative values means backward, positive
 * values means forward.
 * @param motor - Struct of the motor to change the speed
 * @param finalSpeed - The speed to reach
 * @param rampDelay - Delay between each step of the ramp
 */
void speedRamp(Motor* motor, int finalSpeed, int rampDelay) {
  int initialSpeed = motor->speed;
  if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
    Serial.print("Initial speed: ");
    Serial.println(initialSpeed);
    Serial.print("Final speed: ");
    Serial.println(finalSpeed);
  }

  if (initialSpeed > finalSpeed) {
    if (initialSpeed > 0) {
      if (finalSpeed > 0) {
        rampDown(motor, FORWARD, finalSpeed, rampDelay);
      } else {
        rampDown(motor, FORWARD, 0, rampDelay);
        rampUp(motor, BACKWARD, -finalSpeed, rampDelay);
      }
    } else {
      rampUp(motor, BACKWARD, -finalSpeed, rampDelay);
    }
  } else {
    if (initialSpeed > 0) {
      if (finalSpeed > 0) {
        rampUp(motor, FORWARD, finalSpeed, rampDelay);
      } else {
        rampDown(motor, BACKWARD, 0, rampDelay);
        rampUp(motor, FORWARD, finalSpeed, rampDelay);
      }
    } else {
      rampDown(motor, BACKWARD, -finalSpeed, rampDelay);
    }
  }
  motor->speed = finalSpeed;
  setParameter(motor->speedParameter, finalSpeed);
}

/**
 * Accelerate the motor by increading the speed at a given rate, defined by
 * the rampDelay.
 * @param motor - Struct of the motor to change the speed
 * @param direction - Direction of the motor
 * @param finalSpeed - The speed to reach
 * @param rampDelay - Delay between each step of the ramp
 */
void rampUp(Motor* motor, Direction direction, int finalSpeed, int rampDelay) {
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

/**
 * Decelerate the motor by decreasing the speed at a given rate, defined by
 * the rampDelay.
 * @param motor - Struct of the motor to change the speed
 * @param direction - Direction of the motor
 * @param finalSpeed - The speed to reach
 * @param rampDelay - Delay between each step of the ramp
 */
void rampDown(Motor* motor,
              Direction direction,
              int finalSpeed,
              int rampDelay) {
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

/**
 * Function for testing voltage drop on battery when motor speed is varied
 * quickly. Turn the motor on at given speed for 1 second.
 */
void shortFullSpeed(Motor* motor, int speed) {
  analogWrite(motor->pin1, speed);
  analogWrite(motor->pin2, 0);
  vTaskDelay(1000);
  analogWrite(motor->pin1, 0);
  motor->speed = 0;
}