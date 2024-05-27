#pragma once

#include "../state.h"

#include "../tasks/taskEncoders.h"
#include "motorCommands.h"

#define DEFAULT_RAMP_DELAY 1  // delay between each speed increment for ramps
#define MAX_SPEED_COMMAND 255
#define MIN_SPEED_COMMAND -256

/**
 * Parameters for the initialisation of a motor.
 */
struct MotorParams {
  int commandParameter;
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

void updateMotors(Robot* robot, int leftTarget, int rightTarget, int duration);

void motorControl(Motor* motor, Encoder* encoder, int rampStep = 1);