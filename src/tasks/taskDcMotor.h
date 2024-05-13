#pragma once

#include <pwmWrite.h>

struct MotorParams {
  int speedParameter;
  int modeParameter;
  int angleParameter;
  int pin1;
  int pin2;
};

enum MotorMode {
  MOTOR_STOP,            // 0
  MOTOR_CONSTANT_SPEED,  // 1
  MOTOR_MOVE_SECONDS,    // 2
  MOTOR_MOVE_DEGREES,    // 3
  MOTOR_SHORT            // 4
};

void taskDcMotor();
