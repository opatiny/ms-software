#include "./printUtilities.h"
#include "../../src/tasks/motorCommands.h"
#include "./params.h"

void printImu(ImuData* imuData) {
  Serial.print("\t- Acceleration: ");
  Serial.print(imuData->acceleration.x);
  Serial.print(", ");
  Serial.print(imuData->acceleration.y);
  Serial.print(", ");
  Serial.println(imuData->acceleration.z);
  Serial.print("\t- Rotation: ");
  Serial.print(imuData->rotation.x);
  Serial.print(", ");
  Serial.print(imuData->rotation.y);
  Serial.print(", ");
  Serial.println(imuData->rotation.z);
}

void printDistances(int* distances) {
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    Serial.print(distances[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void printMotor(Motor* motor) {
  Serial.print("\t - Pins: ");
  Serial.print(motor->pin1);
  Serial.print(", ");
  Serial.println(motor->pin2);
  Serial.print("\t - Mode: ");
  Serial.println(getParameter(motor->modeParameter));
  Serial.print("\t - Current speed: ");
  Serial.println(motor->speed);
  Serial.print("\t - Target speed: ");
  Serial.println(getParameter(motor->speedParameter));
  Serial.print("\t - Target angle (mode 3 only): ");
  Serial.println(getParameter(motor->angleParameter));
  Serial.print("\t - Encoder counts: ");
  Serial.println(motor->encoderCounts);
}

void printState() {
  Serial.println("Logging current state...\n");
  Serial.println("Left motor:");
  printMotor(&state.leftMotor);
  Serial.println("Right motor:");
  printMotor(&state.rightMotor);
  Serial.print("\nDistances: ");
  printDistances(state.distances);
  Serial.println("\nIMU data");
  printImu(&state.imuData);
}