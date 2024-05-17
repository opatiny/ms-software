#include <globalConfig.h>
#include <utilities/params.h>

#include "motorCommands.h"
#include "printUtilities.h"

/**
 * @brief Print the accelerometer's data.
 */
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

/**
 * @brief Print the distance sensors' data.
 */
void printDistances(int* distances) {
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    Serial.print(distances[i]);
    Serial.print(", ");
  }
  Serial.println();
}

/**
 * @brief Print the data of one of the motors.
 */
void printMotor(Motor* motor) {
  Serial.print("\t - Pins: ");
  Serial.print(motor->pin1);
  Serial.print(", ");
  Serial.println(motor->pin2);
  Serial.print("\t - Mode: ");
  Serial.println(getParameter(motor->modeParameter));
  Serial.print("\t - Current speed: ");
  Serial.println(motor->currentSpeed);
  Serial.print("\t - Target speed: ");
  Serial.println(getParameter(motor->speedParameter));
  Serial.print("\t - Target angle (mode 3 only): ");
  Serial.println(getParameter(motor->angleParameter));
}

/**
 * @brief Print the data of the encoders.
 */
void printEncoder(Encoder* encoder) {
  Serial.print("\t - Pins: ");
  Serial.print(encoder->pin1);
  Serial.print(", ");
  Serial.println(encoder->pin2);
  Serial.print("\t - Counts: ");
  Serial.println(encoder->counts);
}

// todo: enhance print state

/**
 * @brief Print the current state of the robot.
 */
void printState() {
  Serial.println("Logging current state...\n");
  Serial.println("Left motor:");
  printMotor(&robot.leftMotor);
  Serial.println("\nRight motor:");
  printMotor(&robot.rightMotor);
  Serial.println("\nLeft encoder:");
  printEncoder(&robot.leftEncoder);
  Serial.println("\nRight encoder:");
  printEncoder(&robot.rightEncoder);
  Serial.print("\nDistances: ");
  printDistances(robot.distances);
  Serial.println("\nIMU data");
  printImu(&robot.imuData);
}

/**
 * @brief Print the help for the debug modes and the current debug mode.
 */
void printDebug() {
  Serial.println("Use serial parameter U to switch between debug modes.");
  Serial.println("\t0) No debug");
  Serial.println("\t1) Distance sensors");
  Serial.println("\t2) Accelerometer");
  Serial.println("\t3) Battery");
  Serial.println("\t4) Battery for matlab");
  Serial.println("\t5) Encoders");
  Serial.println("\t6) Push button");
  Serial.println("\t7) Motors");
  Serial.println("\t8) RGB LED");
  Serial.println("\t9) Buzzer");

  Serial.print("\nCurrent debug mode: U");
  Serial.println(getParameter(PARAM_DEBUG));
}