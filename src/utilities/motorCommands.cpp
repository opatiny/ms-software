#include <utilities/params.h>

#include "../pinMapping.h"
#include "../state.h"
#include "../tasks/taskRobotMove.h"
#include "kinematics.h"
#include "motorCommands.h"

#define MOTOR_ACC_DURATION 100

void initialiseMotor(Motor* motor, MotorParams* params) {
  // setup the motor parameters
  motor->commandParameter = params->commandParameter;
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
  setParameter(motor->commandParameter, 100);
  setParameter(motor->angleParameter, 90);  // degrees
}

/**
 * @Brief Update the motor speed in a non-blocking way. We use the processor
 * time to estimate how much the speed should be increased.
 * @param motor - Struct of the motor to update.
 * @param duration - Total desired duration of the movement.
 */
void updateMotor(Motor* motor, int target, int duration) {
  if (target == motor->currentCommand) {
    return;
  }

  if (target != motor->previousTargetCommand) {
    // increment necessary per ms
    motor->step = (target - motor->currentCommand) / duration;
    motor->previousTargetCommand = target;
  }

  int dt = millis() - motor->previousTime;
  motor->previousTime = millis();

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

  if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
    Serial.print("Target command: ");
    Serial.print(target);
    Serial.print(", Current command: ");
    Serial.print(motor->currentCommand);
    Serial.print(", Step: ");
    Serial.print(motor->step);
    Serial.print(", Duration: ");
    Serial.println(duration);
  }
}

void updateMotors(Robot* robot, int leftTarget, int rightTarget, int duration) {
  updateMotor(&robot->leftMotor, leftTarget, duration);
  updateMotor(&robot->rightMotor, rightTarget, duration);
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
 * Control the motor based on the current mode and target speed.
 * @param motor - Motor to control
 * @param encoder - Encoder of the motor
 * @param rampStep (opt) - Speed ramp step in ms (default: 1)
 */
void motorControl(Motor* motor, Encoder* encoder, int rampStep) {
  int targetCommand = getParameter(motor->commandParameter);
  int currentMode = getParameter(motor->modeParameter);

  switch (currentMode) {
    case MOTOR_STOP:
      updateMotor(motor, 0, 10);
      break;
    case MOTOR_CONSTANT_SPEED:
      if (motor->previousMode != currentMode ||
          targetCommand != motor->previousTargetCommand) {
        if (getParameter(PARAM_DEBUG) == DEBUG_MOTORS) {
          Serial.println("Motor constant speed mode");
        }
        updateMotor(motor, targetCommand, MOTOR_ACC_DURATION);
      }
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
  vTaskDelay(1);  // smallest delay possible
  motor->previousMode = currentMode;
}