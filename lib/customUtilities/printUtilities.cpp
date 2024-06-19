#include <globalConfig.h>
#include <utilities/params.h>

#include "motorCommands.h"
#include "pidController.h"
#include "printUtilities.h"

void printRegressions(Regressions* regressions, int nbDigits);

/**
 * @brief Print the accelerometer's data.
 */
void printImu(ImuData* imuData) {
  Serial.print("\t- Acceleration [mm/s^2]: ");
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
  Serial.println(motor->currentCommand);
  Serial.print("\t - Target speed: ");
  Serial.println(getParameter(motor->commandParameter));
  Serial.print("\t - Target angle: ");
  Serial.println(getParameter(motor->angleParameter));
  Serial.println("Speed calibration regressions: ");
  printRegressions(&motor->regressions, 5);
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

/**
 * @brief Print the data of the battery.
 */
void printVoltage(VoltageMeasurement* voltageMeasurement) {
  Serial.print("\t - Pin: ");
  Serial.println(voltageMeasurement->pin);
  Serial.print("\t - Voltage [mV]: ");
  Serial.println(getParameter(voltageMeasurement->voltageParameter));
  Serial.print("\t - Warning voltage [mV]: ");
  Serial.println(voltageMeasurement->warningVoltage);
}

void printOdometry(Odometry* odometry) {
  Serial.print("\t - Pose (x,y,theta): ");
  Serial.print(odometry->pose.x);
  Serial.print(", ");
  Serial.print(odometry->pose.y);
  Serial.print(", ");
  Serial.println(odometry->pose.theta);
  Serial.print("\t - Speed (v,omega): ");
  Serial.print(odometry->speed.v);
  Serial.print(", ");
  Serial.println(odometry->speed.omega);
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
  Serial.println("\nOdometry:");
  printOdometry(&robot.odometry);
  Serial.print("\nDistances [mm]: ");
  printDistances(robot.distances);
  Serial.println("\nIMU data");
  printImu(&robot.imuData);
  Serial.println("\nBattery:");
  printVoltage(&robot.battery);
  Serial.println("\nVcc:");
  printVoltage(&robot.vcc);
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
  Serial.println("\t10) Odometry");
  Serial.println("\t11) Speed calibration");
  Serial.println("\t12) Robot control");
  Serial.println("\t13) Motors (only time interval since last call)");
  Serial.println("\t14) Main: print when tasks are up");
  Serial.println("\t15) Wheel speed measurements with different methods");

  Serial.print("\nCurrent debug mode: U");
  Serial.println(getParameter(PARAM_DEBUG));
}

/**
 * @brief Print an array of doubles.
 */
void printArray(double* array, int size, int nbDigits) {
  for (int i = 0; i < size; i++) {
    Serial.print(array[i], nbDigits);
    Serial.print(", ");
  }
  Serial.println();
}

/**
 * @brief Print the positive and negative regressions of a wheel.
 * @param regressions The regressions to print.
 * @param nbDigits The number of decimal digits to print (default is 2).
 */
void printRegressions(Regressions* regressions, int nbDigits) {
  Serial.print("\t- Negative part: ");
  printArray(regressions->pNeg, NB_COEFF, nbDigits);
  Serial.print("\t- Positive part: ");
  printArray(regressions->pPos, NB_COEFF, nbDigits);
}

void printRegressionsForMatlab(Robot* robot, int nbDigits) {
  Serial.println("\n pNegLeft, pPosLeft, pNegRight, pPosRight");
  for (int i = 0; i < NB_COEFF; i++) {
    Serial.print(robot->leftMotor.regressions.pNeg[i], nbDigits);
    Serial.print(", ");
    Serial.print(robot->leftMotor.regressions.pPos[i], nbDigits);
    Serial.print(", ");
    Serial.print(robot->rightMotor.regressions.pNeg[i], nbDigits);
    Serial.print(", ");
    Serial.println(robot->rightMotor.regressions.pPos[i], nbDigits);
  }
  Serial.println();
}

void printControllerParameters(Print* output, PidController* controller) {
  updatePidParameters(controller);
  int nbDigits = 3;
  output->println(F("Serial parameters are Kn * 1000."));
  output->println(F("PID speed controller parameters:"));
  output->print("\t- (AV) Kp: ");
  output->println(controller->params.kp, nbDigits);
  output->print("\t- (AW) Ki: ");
  output->println(controller->params.ki, nbDigits);
  output->print("\t- (AX) Kd: ");
  output->println(controller->params.kd, nbDigits);
}

void showPrintHelp(Print* output) {
  output->println(F("(pc) Print controller parameters"));
  output->println(F("(ps) Print state"));
  output->println(F("(pr) Print regressions (for speed calibration)"));
}

void processPrintCommand(char command,
                         char* paramValue,
                         Print* output) {  // char and char* ??
  switch (command) {
    case 's':
      printState();
      break;
    case 'r':
      printRegressionsForMatlab(&robot, 10);
      break;
    case 'c':

      printControllerParameters(output, &robot.controller.leftSpeedController);
      break;
    default:
      showPrintHelp(output);
  }
}