#include <utilities/params.h>

#include "../pinMapping.h"
#include "../state.h"
#include "../tasks/taskRobotMove.h"
#include "kinematics.h"
#include "motorCommands.h"

// set to 1 for additional debug about speed ramps
#define RAMP_UP_DOWN_DEBUG 0

// prototypes
void rampDown(Motor* motor,
              Direction direction,
              int finalSpeed,
              int rampDelay = DEFAULT_RAMP_DELAY);
void rampUp(Motor* motor,
            Direction direction,
            int finalSpeed,
            int rampDelay = DEFAULT_RAMP_DELAY);

void initialiseMotor(Motor* motor, MotorParams* params) {
  // setup the motor parameters
  motor->speedParameter = params->speedParameter;
  motor->modeParameter = params->modeParameter;
  motor->angleParameter = params->angleParameter;
  motor->pin1 = params->pin1;
  motor->pin2 = params->pin2;
  motor->previousMode = MOTOR_STOP;

  // we will do some PWM on the motor pins
  pinMode(motor->pin1, OUTPUT);
  pinMode(motor->pin2, OUTPUT);

  // initally stop the motor
  analogWrite(motor->pin1, 0);
  analogWrite(motor->pin2, 0);
  setParameter(motor->modeParameter, MOTOR_STOP);

  // initialise motor parameters
  setParameter(motor->speedParameter, 100);
  setParameter(motor->angleParameter, 90);  // degrees
}

/**
 * @brief Rotate the specified motor of a given number of degrees.
 * @param motor Motor to rotate.
 * @param degrees Number of degrees to rotate. Can only be a positive integer.
 * @param speed Speed of the motor (-255 to 255). It is the sign of the speed
 * that defines the direction of the rotation!!
 */
void moveDegrees(Motor* motor, Encoder* encoder, int degrees, int speed) {
  int counts = angleToCounts(degrees);
  int startCounts = encoder->counts;
  int targetCounts = startCounts + counts;

  if (targetCounts > 2 ^ 15 - 1) {
    targetCounts = -2 ^ 15 - 1;
  } else if (targetCounts < -2 ^ 15) {
    targetCounts = -2 ^ 15;
  }
  speedRamp(motor, speed);
  if (speed > 0) {
    while (encoder->counts < targetCounts) {
      vTaskDelay(1);
    }
  } else {
    while (encoder->counts > targetCounts) {
      vTaskDelay(1);
    }
  }
  if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
    Serial.print("true movement in encoder counts: ");

    int trueNbCounts = 0;
    if (speed > 0) {
      trueNbCounts = encoder->counts - startCounts;
    } else {
      trueNbCounts = startCounts - encoder->counts;
    }
    Serial.println(encoder->counts - startCounts);
  }

  speedRamp(motor, 0);
  setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
}

/**
 * Rotate a motor for a given number of seconds at a desired speed.
 * @param motor - Struct of the motor to move.
 * @param seconds - Number of seconds of the movement.
 * @param speed - Speed of the motor.
 */
void moveSeconds(Motor* motor, int seconds, int speed, int rampDelay) {
  speedRamp(motor, speed, rampDelay);
  vTaskDelay(seconds * 1000);
  speedRamp(motor, 0, rampDelay);
  setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
}

/**
 * Stop the given motor using a ramp down.
 * @param motor - Struct of the motor to stop
 */
void stopMotor(Motor* motor) {
  if (motor->currentSpeed > 0) {
    rampDown(motor, FORWARD, 0);
  } else if (motor->currentSpeed < 0) {
    rampDown(motor, BACKWARD, 0);
  }
  motor->currentSpeed = 0;
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
  int initialSpeed = motor->currentSpeed;
  if (initialSpeed == finalSpeed) {
    return;
  }

  if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
    Serial.print("speed ramp: ");
    Serial.print(initialSpeed);
    Serial.print(" -> ");
    Serial.print(finalSpeed);
    Serial.print(" with ");
    Serial.print(rampDelay);
    Serial.println(" ms steps");
  }

  if (initialSpeed > finalSpeed) {
    if (initialSpeed >= 0) {
      if (finalSpeed >= 0) {
        rampDown(motor, FORWARD, finalSpeed, rampDelay);
      } else {
        rampDown(motor, FORWARD, 0, rampDelay);
        motor->currentSpeed = 0;
        rampUp(motor, BACKWARD, -finalSpeed, rampDelay);
      }
    } else {
      rampUp(motor, BACKWARD, -finalSpeed, rampDelay);
    }
  } else {
    if (initialSpeed <= 0) {
      if (finalSpeed <= 0) {
        rampDown(motor, BACKWARD, -finalSpeed, rampDelay);
      } else {
        rampDown(motor, BACKWARD, 0, rampDelay);
        motor->currentSpeed = 0;
        rampUp(motor, FORWARD, finalSpeed, rampDelay);
      }
    } else {
      rampUp(motor, FORWARD, finalSpeed, rampDelay);
    }
  }
  motor->currentSpeed = finalSpeed;
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
  int initialSpeed = motor->currentSpeed;
  if (initialSpeed < 0) {
    initialSpeed = -initialSpeed;
  }
  if (RAMP_UP_DOWN_DEBUG) {
    Serial.print("Ramp up: ");
    Serial.print(initialSpeed);
    Serial.print(" -> ");
    Serial.print(finalSpeed);
    Serial.print(", direction: ");
    Serial.println(direction);
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
  int initialSpeed = motor->currentSpeed;
  if (initialSpeed < 0) {
    initialSpeed = -initialSpeed;
  }
  if (RAMP_UP_DOWN_DEBUG) {
    Serial.print("Ramp down: ");
    Serial.print(initialSpeed);
    Serial.print(" -> ");
    Serial.print(finalSpeed);
    Serial.print(", direction: ");
    Serial.println(direction);
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
 * quickly. Turn the motor on at given speed for `delaySec` seconds. No ramp is
 * used in this function!!
 */
void shortFullSpeed(Motor* motor, int speed, int delaySec) {
  if (speed < 0) {
    speed = -speed;
  }
  analogWrite(motor->pin1, speed);
  analogWrite(motor->pin2, 0);
  vTaskDelay(delaySec * 1000);
  analogWrite(motor->pin1, 0);
  motor->currentSpeed = 0;
}

/**
 * Control the motor based on the current mode and target speed.
 * @param motor - Motor to control
 * @param encoder - Encoder of the motor
 * @param rampStep (opt) - Speed ramp step in ms (default: 1)
 */
void motorControl(Motor* motor, Encoder* encoder, int rampStep) {
  int targetSpeed = getParameter(motor->speedParameter);
  int currentMode = getParameter(motor->modeParameter);

  switch (currentMode) {
    case MOTOR_STOP:
      stopMotor(motor);
      break;
    case MOTOR_CONSTANT_SPEED:
      if (motor->previousMode != currentMode ||
          targetSpeed != motor->currentSpeed) {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor constant speed mode");
        }
        speedRamp(motor, targetSpeed, getParameter(PARAM_MOTOR_RAMP_STEP));
      }
      break;
    case MOTOR_MOVE_SECONDS: {
      int delaySeconds = 1;
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.print("Move for ");
        Serial.print(delaySeconds);
        Serial.print(" seconds at speed ");
        Serial.println(targetSpeed);
      }
      moveSeconds(motor, delaySeconds, targetSpeed,
                  getParameter(PARAM_MOTOR_RAMP_STEP));
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.println("End of movement");
      }
      break;
    }
    case MOTOR_MOVE_DEGREES: {
      int degrees = getParameter(motor->angleParameter);
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        Serial.print("Move for ");
        Serial.print(degrees);
        Serial.print(" degrees at speed ");
        Serial.println(targetSpeed);
      }
      moveDegrees(motor, encoder, degrees, targetSpeed);
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
      }
      break;
    }
    case MOTOR_SHORT: {
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        int delay = 1;
        Serial.println("Short pulse mode (only for debug)");
        Serial.print("Motor will spin for ");
        Serial.print(delay);
        Serial.print(" seconds at speed ");
        Serial.println(targetSpeed);
      }
      shortFullSpeed(motor, targetSpeed, 1);
      setParameter(motor->modeParameter, MOTOR_STOP);
      break;
    }
    default:
      Serial.println("Unknown motor mode");
      setParameter(motor->modeParameter, MOTOR_STOP);
      break;
  }
  motor->previousMode = currentMode;
}