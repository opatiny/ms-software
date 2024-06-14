#pragma once

#include <state.h>

#define DEFAULT_ACC_DURATION 100  // default duration of the acceleration [ms]
#define MOTOR_STOP_DURATION 10    // time to stop [ms]

#define MAX_SPEED_COMMAND 255
#define MIN_SPEED_COMMAND -256

/**
 * Parameters for the initialisation of a motor.
 */
struct MotorParams {
  int commandParameter;
  int modeParameter;
  int angleParameter;
  int accDurationParameter;
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

void updateMotor(Motor* motor, int target, uint32_t duration);

void updateMotors(Robot* robot,
                  int leftTarget,
                  int rightTarget,
                  uint32_t duration);

void stopMotors(Robot* robot);

void motorControl(Motor* motor, Encoder* encoder);