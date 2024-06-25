#include <globalConfig.h>
#include <utilities/params.h>

#include "motorCommands.h"
#include "pidController.h"
#include "printUtilities.h"

void printRegressions(Print* output, Regressions* regressions, int nbDigits);

/**
 * @brief Print the accelerometer's data.
 */
void printImu(Print* output, ImuData* imuData) {
  output->print("\t- Acceleration [mm/s^2]: ");
  output->print(imuData->acceleration.x);
  output->print(", ");
  output->print(imuData->acceleration.y);
  output->print(", ");
  output->println(imuData->acceleration.z);
  output->print("\t- Rotation: ");
  output->print(imuData->rotation.x);
  output->print(", ");
  output->print(imuData->rotation.y);
  output->print(", ");
  output->println(imuData->rotation.z);
}

/**
 * @brief Print the distance sensors' data.
 */
void printDistances(Print* output, int* distances) {
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    output->print(distances[i]);
    output->print(", ");
  }
  output->println();
}

/**
 * @brief Print the data of one of the motors.
 */
void printMotor(Print* output, Motor* motor) {
  output->print("\t - Pins: ");
  output->print(motor->pin1);
  output->print(", ");
  output->println(motor->pin2);
  output->print("\t - Mode: ");
  output->println(getParameter(motor->modeParameter));
  output->print("\t - Current speed: ");
  output->println(motor->currentCommand);
  output->print("\t - Target speed: ");
  output->println(getParameter(motor->commandParameter));
  output->print("\t - Target angle: ");
  output->println(getParameter(motor->angleParameter));
  output->println("Speed calibration regressions: ");
  printRegressions(output, &motor->regressions, 5);
}

/**
 * @brief Print the data of the encoders.
 */
void printEncoder(Print* output, Encoder* encoder) {
  output->print("\t - Pins: ");
  output->print(encoder->pin1);
  output->print(", ");
  output->println(encoder->pin2);
  output->print("\t - Counts: ");
  output->println(encoder->counts);
}

/**
 * @brief Print the data of the battery.
 */
void printVoltage(Print* output, VoltageMeasurement* voltageMeasurement) {
  output->print("\t - Pin: ");
  output->println(voltageMeasurement->pin);
  output->print("\t - Voltage [mV]: ");
  output->println(getParameter(voltageMeasurement->voltageParameter));
  output->print("\t - Warning voltage [mV]: ");
  output->println(voltageMeasurement->warningVoltage);
}

void printOdometry(Print* output, Odometry* odometry) {
  output->print("\t - Pose (x,y,theta): ");
  output->print(odometry->pose.x);
  output->print(", ");
  output->print(odometry->pose.y);
  output->print(", ");
  output->println(odometry->pose.theta);
  output->print("\t - Speed (v,omega): ");
  output->print(odometry->speed.v);
  output->print(", ");
  output->println(odometry->speed.omega);
}

// todo: enhance print state

/**
 * @brief Print the current state of the robot.
 */
void printState(Print* output) {
  output->print("Logging current state...\n");
  output->println("Left motor:");
  printMotor(output, &robot.leftMotor);
  output->println("\nRight motor:");
  printMotor(output, &robot.rightMotor);
  output->println("\nLeft encoder:");
  printEncoder(output, &robot.leftEncoder);
  output->println("\nRight encoder:");
  printEncoder(output, &robot.rightEncoder);
  output->println("\nOdometry:");
  printOdometry(output, &robot.odometry);
  output->print("\nDistances [mm]: ");
  printDistances(output, robot.distances);
  output->println("\nIMU data");
  printImu(output, &robot.imuData);
  output->print("\nBattery:");
  printVoltage(output, &robot.battery);
  output->println("\nVcc:");
  printVoltage(output, &robot.vcc);
}

/**
 * @brief Print the help for the debug modes and the current debug mode.
 */
void printDebug(Print* output) {
  output->println("Use serial parameter U to switch between debug modes.");
  output->println("\t0) No debug");
  output->println("\t1) Distance sensors");
  output->println("\t2) Accelerometer");
  output->println("\t3) Battery");
  output->println("\t4) Battery for matlab");
  output->println("\t5) Encoders");
  output->println("\t6) Push button");
  output->println("\t7) Motors");
  output->println("\t8) RGB LED");
  output->println("\t9) Buzzer");
  output->println("\t10) Odometry");
  output->println("\t11) Speed calibration");
  output->println("\t12) Robot control");
  output->println("\t13) Motors (only time interval since last call)");
  output->println("\t14) Main: print when tasks are up");
  output->println("\t15) Wheel speed measurements with different methods");

  output->print("\nCurrent debug mode: U");
  output->println(getParameter(PARAM_DEBUG));
}

/**
 * @brief Print an array of doubles.
 */
void printArray(Print* output, double* array, int size, int nbDigits) {
  for (int i = 0; i < size; i++) {
    output->print(array[i], nbDigits);
    output->print(", ");
  }
  output->println();
}

/**
 * @brief Print the positive and negative regressions of a wheel.
 * @param regressions The regressions to print.
 * @param nbDigits The number of decimal digits to print (default is 2).
 */
void printRegressions(Print* output, Regressions* regressions, int nbDigits) {
  output->print("\t- Negative part: ");
  printArray(output, regressions->pNeg, NB_COEFF, nbDigits);
  output->print("\t- Positive part: ");
  printArray(output, regressions->pPos, NB_COEFF, nbDigits);
}

void printRegressionsForMatlab(Print* output, Robot* robot, int nbDigits) {
  output->println("\n pNegLeft, pPosLeft, pNegRight, pPosRight");
  for (int i = 0; i < NB_COEFF; i++) {
    output->print(robot->leftMotor.regressions.pNeg[i], nbDigits);
    output->print(", ");
    output->print(robot->leftMotor.regressions.pPos[i], nbDigits);
    output->print(", ");
    output->print(robot->rightMotor.regressions.pNeg[i], nbDigits);
    output->print(", ");
    output->println(robot->rightMotor.regressions.pPos[i], nbDigits);
  }
  output->println();
}

void printControllerParameters(Print* output, Robot* robot) {
  PidController* wheel = &robot->navigation.wheelsSpeedController.left;
  PidController* linear = &robot->navigation.robotSpeedController.linear;
  PidController* angular = &robot->navigation.robotSpeedController.angular;
  updatePidParameters(wheel);
  updatePidParameters(linear);
  updatePidParameters(angular);

  int nbDigits = 3;
  output->println(F("Serial parameters are Kn * 1000."));
  output->println(F("PID wheels speed controller parameters:"));
  output->print("\t- (BA) Kp: ");
  output->println(wheel->params.kp, nbDigits);
  output->print("\t- (BB) Ki: ");
  output->println(wheel->params.ki, nbDigits);
  output->print("\t- (BC) Kd: ");
  output->println(wheel->params.kd, nbDigits);

  output->println(F("PID robot linear speed controller parameters:"));
  output->print("\t- (BD) Kp: ");
  output->println(linear->params.kp, nbDigits);
  output->print("\t- (BE) Ki: ");
  output->println(linear->params.ki, nbDigits);
  output->print("\t- (BF) Kd: ");
  output->println(linear->params.kd, nbDigits);

  output->println(F("PID robot angular speed controller parameters:"));
  output->print("\t- (BG) Kp: ");
  output->println(angular->params.kp, nbDigits);
  output->print("\t- (BH) Ki: ");
  output->println(angular->params.ki, nbDigits);
  output->print("\t- (BI) Kd: ");
  output->println(angular->params.kd, nbDigits);
}

void showPrintHelp(Print* output) {
  output->println(F("(pc) Print controllers parameters"));
  output->println(F("(ps) Print state"));
  output->println(F("(pr) Print regressions (for speed calibration)"));
}

void processPrintCommand(char command,
                         char* paramValue,
                         Print* output) {  // char and char* ??
  switch (command) {
    case 's':
      printState(output);
      break;
    case 'r':
      printRegressionsForMatlab(output, &robot, 10);
      break;
    case 'c':
      printControllerParameters(output, &robot);
      break;
    default:
      showPrintHelp(output);
  }
}