#include "../state.h"
#include "motorCommands.h"

void robotMove(int speed) {
  speedRamp(&state.leftMotor, speed);
  speedRamp(&state.rightMotor, speed);
}

void robotStop() {
  stopMotor(&state.leftMotor);
  stopMotor(&state.rightMotor);
}

void robotTurnInPlace(int speed) {
  speedRamp(&state.leftMotor, speed);
  speedRamp(&state.rightMotor, -speed);
}
