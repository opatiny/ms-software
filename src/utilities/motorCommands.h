#pragma once

#include "../state.h"

#include "../tasks/taskEncoders.h"
#include "motorCommands.h"

#define DEFAULT_RAMP_DELAY 1  // delay between each speed increment for ramps

/**
 * Parameters for the initialisation of a motor.
 */
struct MotorParams {
  int speedParameter;
  int modeParameter;
  int angleParameter;
  int pin1;
  int pin2;
};

/**
 * Direction in which the motor spins.
 *
 * - BACKWARD: 0
 * - FORWARD: 1
 */
enum Direction { BACKWARD, FORWARD };

void initialiseMotor(Motor* motor, MotorParams* params);

void stopMotor(Motor* motor);

void speedRamp(Motor* motor,
               int finalSpeed,
               int rampDelay = DEFAULT_RAMP_DELAY);
void shortFullSpeed(Motor* motor, int speed, int delaySec = 1);

void moveSeconds(Motor* motor,
                 int seconds,
                 int speed,
                 int rampDelay = DEFAULT_RAMP_DELAY);

void moveDegrees(Motor* motor, Encoder* encoder, int degrees, int speed);

void motorControl(Motor* motor, Encoder* encoder);