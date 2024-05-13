#pragma once

#include "taskEncoders.h"

// motors and wheels properties
#define GEAR_RATIO 30
#define COUNTS_PER_REV 12
#define WHEEL_DIAMETER 32     // in mm
#define WHEEL_BASE 100        // in mm, distance between centers of wheels
#define DEFAULT_RAMP_DELAY 1  // delay between each speed increment for ramps

// motors structures with all data
/**
 * Motor structure.
 *  - speedParameter: Serial parameter for the target speed of the motor.
 * - modeParameter: Serial parameter for the mode of the motor.
 * - previousMode: Previous mode of the motor.
 * - angleParameter: Serial parameter for the angle of the motor.
 * - encoderCounts: Number of counts of the encoder since the robot was turned
 * on.
 * - speed: Current speed of the motor.
 */
struct Motor {
  int speedParameter;
  int modeParameter;
  int previousMode;
  int angleParameter;
  Encoder encoderCounts;
  int currentSpeed;
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

void stopMotor(Motor* motor);

void speedRamp(Motor* motor,
               int finalSpeed,
               int rampDelay = DEFAULT_RAMP_DELAY);
void shortFullSpeed(Motor* motor, int speed, int delaySec = 1);

void moveSeconds(Motor* motor,
                 int seconds,
                 int speed,
                 int rampDelay = DEFAULT_RAMP_DELAY);

void moveDegrees(Motor* motor, int degrees, int speed);