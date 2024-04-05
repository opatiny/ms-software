#pragma once

#include <pwmWrite.h>

#define MOTOR_LEFT_PIN1 D9
#define MOTOR_LEFT_PIN2 D10

enum MotorMode {
  MOTOR_STOP,
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_RAMP,
  MOTOR_SHORT
};
void taskDcMotor();
