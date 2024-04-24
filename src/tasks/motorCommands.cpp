#include "./motorCommands.h"
#include <utilities/params.h>

Motor leftMotor = {
  speedParameter : PARAM_MOTOR_LEFT_SPEED,
  modeParameter : PARAM_MOTOR_LEFT_MODE,
  encoderParameter : PARAM_ENCODER_LEFT,
  speed : 0,
  pin1 : MOTOR_LEFT_PIN1,
  pin2 : MOTOR_LEFT_PIN2
};

Motor rightMotor = {
  speedParameter : PARAM_MOTOR_RIGHT_SPEED,
  modeParameter : PARAM_MOTOR_RIGHT_MODE,
  encoderParameter : PARAM_ENCODER_RIGHT,
  speed : 0,
  pin1 : MOTOR_RIGHT_PIN1,
  pin2 : MOTOR_RIGHT_PIN2
};

/**
 * @brief Convert nb of encoder counts to an angle in degrees.
 * todo: Is the return type int a problem?
 */
int countsToAngle(int counts) {
  return counts * 360 / (12 * GEAR_RATIO);
}

/**
 * @brief Convert an angle in degrees to nb of encoder counts.
 */
int angleToCounts(int angle) {
  return angle * 12 * GEAR_RATIO / 360;
}

/**
 * @brief Rotate the specified motor of a given number of degrees.
 * @param motor Motor to rotate.
 * @param degrees Number of degrees to rotate.
 * @param speed Speed of the motor (0 to 255)
 */
void moveDegrees(Motor motor, int degrees, int speed) {
  int counts = angleToCounts(degrees);
  int startCounts = getParameter(motor.encoderParameter);
  int targetCounts = startCounts + counts;
}

/**
 * Rotate a motor for a given number of seconds at a desired speed.
 * @param motor - Struct of the motor to move.
 * @param seconds - Number of seconds of the movement.
 * @param speed - Speed of the motor.
 */
void moveSeconds(Motor motor, int seconds, int speed) {
  speedRamp(&motor, speed);
  vTaskDelay(seconds * 1000);
  stopMotor(&motor);
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
 * quickly. Turn the motor on at given speed for 1 second. No ramp is used in
 * this function!!
 */
void shortFullSpeed(Motor* motor, int speed) {
  if (speed < 0) {
    speed = -speed;
  }
  analogWrite(motor->pin1, speed);
  analogWrite(motor->pin2, 0);
  vTaskDelay(1000);
  analogWrite(motor->pin1, 0);
  motor->speed = 0;
}