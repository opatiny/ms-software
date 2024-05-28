/**
 * Motors can be controlled independently when the robot is in mode AO6
 *
 * Use the serial parameters to act on the motors.
 *
 *   - motor mode: commands AC (left) and AD (right)
 *   - motor speed: commands AA (left) and AB (right)
 *     - speed is in range [-255,255]
 *     - 0 means stop
 *     - positive values means forward
 *     - negative values means backward
 *  - motor angle: commands AJ (left) and AK (right)
 *
 * Debug: U7
 */

#include <utilities/params.h>

#include "../pinMapping.h"
#include "../state.h"
#include "../tasks/taskRobotMove.h"
#include "kinematics.h"
#include "motorCommands.h"

// logging all of the variables adds some unnexpected delays
// when we only log dt, we see that it works as expected
#define DT_DEBUG 1

// todo: handle if this is a big value, like 1000
#define MOTOR_STOP_DURATION 10  // time to stop [ms]

void initialiseMotor(Motor* motor, MotorParams* params) {
  // setup the motor parameters
  motor->commandParameter = params->commandParameter;
  motor->modeParameter = params->modeParameter;
  motor->angleParameter = params->angleParameter;
  motor->accDurationParameter = params->accDurationParameter;
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
  setParameter(motor->commandParameter, 100);
  setParameter(motor->angleParameter, 90);                          // degrees
  setParameter(motor->accDurationParameter, DEFAULT_ACC_DURATION);  // ms
}

/**
 * @brief Update the motor speed in a non-blocking way. We use the processor
 * time to estimate how much the speed should be increased.
 * @param motor - Struct of the motor to update.
 * @param duration - Total desired duration of the movement.
 */
void updateMotor(Motor* motor, int target, int duration) {
  if (target == motor->currentCommand) {
    return;
  }
  int dt = 1;
  int newTime = millis();
  if (motor->previousTargetCommand == target) {
    dt = newTime - motor->previousTime;
  }
  motor->previousTime = newTime;

  if (target != motor->previousTargetCommand) {
    // increment necessary per ms: if duration is too big, step would be 0, so
    // we set the min step to be 1 or -1
    int diff = target - motor->currentCommand;
    int idealStep = diff / duration;

    if (idealStep == 0 && diff > 0) {
      motor->step = 1;
    } else if (idealStep == 0 && diff <= 0) {
      motor->step = -1;
    } else {
      motor->step = idealStep;
    }
    motor->previousTargetCommand = target;
  }

  if (dt == 0) {
    return;
  }

  int newIncrement = motor->step * dt;
  if (abs(newIncrement) < abs(target - motor->currentCommand)) {
    motor->currentCommand += newIncrement;
  } else {
    motor->currentCommand = target;
  }

  if (motor->currentCommand >= 0) {
    analogWrite(motor->pin1, motor->currentCommand);
    analogWrite(motor->pin2, 0);
  } else {
    analogWrite(motor->pin1, 0);
    analogWrite(motor->pin2, -motor->currentCommand);
  }

  if (DT_DEBUG) {
    Serial.print("dt: ");
    Serial.println(dt);
  }

  if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
    Serial.print("Target command: ");
    Serial.print(target);
    Serial.print(", Current command: ");
    Serial.print(motor->currentCommand);
    Serial.print(", Step: ");
    Serial.print(motor->step);
    Serial.print(", Duration: ");
    Serial.print(duration);
    Serial.print(", dt: ");
    Serial.println(dt);
  }
}

void updateMotors(Robot* robot, int leftTarget, int rightTarget, int duration) {
  updateMotor(&robot->leftMotor, leftTarget, duration);
  updateMotor(&robot->rightMotor, rightTarget, duration);
}

void stopMotor(Motor* motor) {
  updateMotor(motor, 0, MOTOR_STOP_DURATION);
}

void stopMotors(Robot* robot) {
  stopMotor(&robot->leftMotor);
  stopMotor(&robot->rightMotor);
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
  motor->currentCommand = 0;
}

/**
 * @brief Control the motor based on the current mode and target speed command.
 * These two variables are defined in the serial parameters.
 * @param motor - Motor to control.
 * @param encoder - Encoder of the motor. // todo: remove this parameter if
 * useless
 */
void motorControl(Motor* motor, Encoder* encoder) {
  int targetCommand = getParameter(motor->commandParameter);
  int currentMode = getParameter(motor->modeParameter);

  switch (currentMode) {
    case MOTOR_STOP:
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS &&
          motor->previousMode != currentMode) {
        Serial.println("Stopping motor");
      }
      stopMotor(motor);
      break;
    case MOTOR_CONSTANT_SPEED:
      if (motor->previousMode != currentMode ||
          targetCommand != motor->previousTargetCommand) {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor constant speed mode");
        }
      }
      updateMotor(motor, targetCommand, getParameter(PARAM_MOTOR_ACC_DURATION));
      break;
    case MOTOR_SHORT: {
      if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
        int delay = 1;
        Serial.println("Short pulse mode (only for debug)");
        Serial.print("Motor will spin for ");
        Serial.print(delay);
        Serial.print(" seconds at speed ");
        Serial.println(targetCommand);
      }
      shortFullSpeed(motor, targetCommand, 1);
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