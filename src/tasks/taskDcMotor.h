#pragma once

#include <pwmWrite.h>

#define MOTOR_LEFT_PIN1 D9
#define MOTOR_LEFT_PIN2 D10

enum MotorMode {
  MOTOR_STOP,      // 0
  MOTOR_FORWARD,   // 1
  MOTOR_BACKWARD,  // 2
  MOTOR_RAMP,      // 3
  MOTOR_SHORT      // 4
};
void taskDcMotor();
