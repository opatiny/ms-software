#pragma once

// motor pins
#define MOTOR_LEFT_PIN1 D9
#define MOTOR_LEFT_PIN2 D10
#define MOTOR_RIGHT_PIN1 1
#define MOTOR_RIGHT_PIN2 2

// motors and wheels properties
#define GEAR_RATIO 30
#define COUNTS_PER_REV 12
#define WHEEL_DIAMETER 32  // in mm
#define WHEEL_BASE 100     // in mm, distance between centers of wheels

// motors parameters
struct Motor {
  int speedParameter;
  int modeParameter;
  int encoderParameter;
  int speed;
  int pin1;
  int pin2;
};

extern Motor leftMotor;
extern Motor rightMotor;

void moveDegrees(Motor motor, int degrees, int speed);