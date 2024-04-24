#pragma once

#include <pwmWrite.h>

enum MotorMode {
  MOTOR_STOP,            // 0
  MOTOR_CONSTANT_SPEED,  // 1
  MOTOR_MOVE_DEGREES,    // 2
  MOTOR_SHORT            // 3
};

void taskDcMotor();
