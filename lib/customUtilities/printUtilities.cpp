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
  output->print("\t- Acceleration [m/s^2]: ");
  output->print(imuData->acceleration.x);
  output->print(", ");
  output->print(imuData->acceleration.y);
  output->print(", ");
  output->println(imuData->acceleration.z);
  output->print("\t- Rotation [rad/s]: ");
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
  // output->print("\t - Pins: ");
  // output->print(motor->pin1);
  // output->print(", ");
  // output->println(motor->pin2);
  // output->print("\t - Mode: ");
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
  // output->print("\t - Pins: ");
  // output->print(encoder->pin1);
  // output->print(", ");
  // output->println(encoder->pin2);
  output->print("\t - Counts: ");
  output->println(encoder->counts);
}

/**
 * @brief Print the data of the battery.
 */
void printVoltage(Print* output, VoltageMeasurement* voltageMeasurement) {
  // output->print("\t - Pin: ");
  // output->println(voltageMeasurement->pin);
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
  output->println("\nBattery:");
  printVoltage(output, &robot.battery);
  output->println("\nVcc:");
  printVoltage(output, &robot.vcc);
}

/**
 * @brief Print the help for the debug modes and the current debug mode.
 */
void printDebug(Print* output) {
  output->println("Use serial parameter A to switch between debug modes.");
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
  output->println(
      "\t16) Robot's acceleration measurement (only with obstacle avoidance "
      "mode)");
  output->print("\nCurrent debug mode: A");
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

/**
 * Print out a PID controller.
 * @param output The output to print to.
 * @param controller The controller structure.
 * @param paramNames The names of the serial parameters.
 * @param mode The mode of the controller (on or off).
 * @param unit The unit of the target value.
 */
void printController(Print* output,
                     PidController* controller,
                     const char* paramNames[5],
                     bool mode,
                     int targetParam,
                     const char* unit) {
  updatePidParameters(controller);
  const int nbDigits = ceil(log10(controller->factor));
  output->printf("\t- (%s) Mode: %s\n", paramNames[0], mode ? "ON" : "OFF");
  output->printf("\t- (%s) Target: %i %s\n\n", paramNames[1],
                 getParameter(targetParam), unit);

  output->printf("\t- Factor: %i\n", controller->factor);
  output->printf("\t- (%s) Kp: %.*f\n", paramNames[2], nbDigits,
                 controller->params.kp);
  output->printf("\t- (%s) Ki: %.*f\n", paramNames[3], nbDigits,
                 controller->params.ki);
  output->printf("\t- (%s) Kd: %.*f\n\n", paramNames[4], nbDigits,
                 controller->params.kd);
}

void printControllers(Print* output, Robot* robot) {
  PidController* wheel = &robot->navigation.wheelsSpeedController.left;
  PidController* linear = &robot->navigation.robotSpeedController.linear;
  PidController* angular = &robot->navigation.robotSpeedController.angular;

  bool linMode = getParameter(
      robot->navigation.robotSpeedController.modeParameters.linearController);
  bool angMode = getParameter(
      robot->navigation.robotSpeedController.modeParameters.angularController);

  const int nbParams = 5;
  // mode, target, Kp, Ki, Kd
  const char* wheelsPidNames[nbParams] = {"none", "Q", "BD", "BE", "BF"};
  const char* robotLinearPidNames[nbParams] = {"BA", "R", "BG", "BH", "BI"};
  const char* robotAngularPidNames[nbParams] = {"BB", "S", "BJ", "BK", "BL"};

  output->println(F("PID wheels speed controller:"));
  printController(output, wheel, wheelsPidNames, 1, PARAM_ROBOT_WHEELS_SPEED,
                  "rpm");

  output->println(F("PID robot linear speed controller:"));
  printController(output, linear, robotLinearPidNames, linMode,
                  PARAM_ROBOT_SPEED_LIN, "mm/s");

  output->println(F("PID robot angular speed controller:"));
  printController(output, angular, robotAngularPidNames, angMode,
                  PARAM_ROBOT_SPEED_ANG, "deg/s");
}

void showPrintHelp(Print* output) {
  output->println(F("(pc) Print controllers parameters"));
  output->println(F("(ps) Print state"));
  output->println(F("(pr) Print regressions (for speed calibration)"));
}

void processPrintCommand(char command, char* paramValue, Print* output) {
  switch (command) {
    case 's':
      printState(output);
      break;
    case 'r':
      printRegressionsForMatlab(output, &robot, 10);
      break;
    case 'c':
      printControllers(output, &robot);
      break;
    default:
      showPrintHelp(output);
  }
}

/**
 * @brief Print the robot's control modes.
 */
void printRobotModes(Print* output) {
  output->println("Use serial parameter T to choose robot mode.");
  output->println("\t0) Stop robot");
  output->println("\t1) Same PWM command for both wheels");
  output->println("\t2) Wheel speed with feedforward control");
  output->println("\t6) Wheel speed control + obstacle avoidance");
  output->println("\t7) Control each wheel separately");
  output->println("\t8) PID on wheel speeds to move straight");
  output->println("\t9) PID on robot speed (linear and angular)");

  output->print("\nCurrent robot mode: T");
  output->println(getParameter(PARAM_ROBOT_MODE));
}

/**
 * @brief Print the buzzer's modes.
 */
void printBuzzerModes(Print* output) {
  output->println("Use serial parameter D to chose buzzer mode.");
  output->println("\t0) Buzzer off");
  output->println("\t1) Single note");
  output->println("\t2) Alarm");
  output->println("\t3) Scale");
  output->println("\t4) Boot");

  output->print("\nCurrent buzzer mode: D");
  output->println(getParameter(PARAM_BUZZER_MODE));
}

/**
 * @brief Print the RGB LED's modes.
 */
void printLedModes(Print* output) {
  output->println("Use serial parameter G to chose LED mode.");
  output->println("\t0) LED off");
  output->println("\t1) LED on");
  output->println("\t2) Blink");

  output->print("\nCurrent LED mode: G");
  output->println(getParameter(PARAM_RGB_LED_MODE));
}

void showModesHelp(Print* output) {
  output->println(F("(mb) Buzzer modes"));
  output->println(F("(ml) RGB LED modes"));
  output->println(F("(mr) Robot modes"));
}

void processModesCommand(char command, char* paramValue, Print* output) {
  switch (command) {
    case 'b':
      printBuzzerModes(output);
      break;
    case 'l':
      printLedModes(output);
      break;
    case 'r':
      printRobotModes(output);
      break;
    default:
      showModesHelp(output);
  }
}