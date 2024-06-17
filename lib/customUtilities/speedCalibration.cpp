#include <eeprom.h>
#include <globalConfig.h>
#include <kinematics.h>
#include <state.h>
#include <timeUtilities.h>
#include <utilities/params.h>

#include "printUtilities.h"
#include "robotModes.h"
#include "speedCalibration.h"

/**
 * Initialise the calibration data structure.
 * @param data: calibration data structure to initialise.
 */
void initialiseCalibrationData(CalibrationData* data) {
  data->command = MIN_MOTOR_COMMAND;
  setParameter(PARAM_CALIBRATION_MODE, CALIBRATION_OFF);
  setParameter(PARAM_CALIBRATION_STEP, 5);
}

/**
 * Non-blocking calibration function (can be stopped at any time).
 * Allows to find the polynomial relationship between the motor command and the
 * wheel speed for both wheels.
 * @param data: calibration data structure. Contains the commands as well as the
 * left and right measured speeds in rpm.
 * @param leftRegressions: regressions for the left wheel
 * @param rightRegressions: regressions for the right wheel
 */
void wheelSpeedCalibration(CalibrationData* data,
                           Regressions* leftRegressions,
                           Regressions* rightRegressions) {
  double currentTime = getSeconds();
  if (data->command == MIN_MOTOR_COMMAND) {
    Serial.println("Speed calibration started...\n");
    Serial.print("Calibration command step: ");
    Serial.println(getParameter(PARAM_CALIBRATION_STEP));
    setParameter(PARAM_ROBOT_MODE, ROBOT_MOVE_SAME_COMMAND);
  }

  if (getParameter(PARAM_DEBUG) == DEBUG_SPEED_CALIBRATION) {
    Serial.print(currentTime, 3);
    Serial.print(", ");
    Serial.print(getParameter(PARAM_BATTERY_VOLTAGE));
    Serial.print(", ");
    Serial.print(data->command);
    Serial.print(", ");
    Serial.print(robot.leftMotor.wheelSpeed);
    Serial.print(", ");
    Serial.println(robot.rightMotor.wheelSpeed);
  }

  data->commands[data->index] = data->command;
  data->leftSpeeds[data->index] = robot.leftMotor.wheelSpeed;
  data->rightSpeeds[data->index] = robot.rightMotor.wheelSpeed;

  data->command += getParameter(PARAM_CALIBRATION_STEP);
  // end condition for the calibration
  if (data->command > MAX_MOTOR_COMMAND) {
    // find the regressions
    double* x = data->commands;
    double* y = data->leftSpeeds;
    getRegressions(leftRegressions, x, y, CALIBRATION_SPEED_LIMIT);
    x = data->commands;
    y = data->rightSpeeds;
    getRegressions(rightRegressions, x, y, CALIBRATION_SPEED_LIMIT);

    Serial.println("Saving regressions to EEPROM...");
    saveWheelsRegressions(leftRegressions, rightRegressions);

    Serial.println("\nLeft wheel regressions:");
    printRegressions(leftRegressions, 10);
    Serial.println("Right wheel regressions:");
    printRegressions(rightRegressions, 10);

    Serial.println("Speed calibration finished.");
    setParameter(PARAM_CALIBRATION_MODE, CALIBRATION_OFF);
    clearCalibrationData(data);
    setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    setParameter(PARAM_ROBOT_COMMAND, 0);
    return;
  }
  setParameter(PARAM_ROBOT_COMMAND, data->command);
  data->index++;
}

/**
 * Clear the calibration data.
 * @param data: calibration data structure to clear.
 */
void clearCalibrationData(CalibrationData* data) {
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    data->commands[i] = 0;
    data->leftSpeeds[i] = 0;
    data->rightSpeeds[i] = 0;
  }
  data->index = 0;
  data->command = MIN_MOTOR_COMMAND;
}

void initialiseTestCalibrationData(TestCalibrationData* data) {
  data->speedStep = CALIBRATION_TEST_SPEED_STEP;
  data->speed = -CALIBRATION_SPEED_LIMIT;
}

void testCalibration(Robot* robot, TestCalibrationData* data) {
  double currentTime = getSeconds();
  if (data->speed == -CALIBRATION_SPEED_LIMIT) {
    Serial.println("Speed calibration test started...\n");
    Serial.print("Speed step: ");
    Serial.println(data->speedStep);
    setParameter(PARAM_ROBOT_MODE, ROBOT_MOVE);
    setParameter(PARAM_ROBOT_SPEED, data->speed);
    Serial.println("\ntime, batteryVoltage, speed, leftSpeed, rightSpeed");
  }

  Serial.print(currentTime, 3);
  Serial.print(", ");
  Serial.print(getParameter(PARAM_BATTERY_VOLTAGE));
  Serial.print(", ");
  Serial.print(data->speed);
  Serial.print(", ");
  Serial.print(robot->leftMotor.wheelSpeeds.highSpeed);
  Serial.print(", ");
  Serial.print(robot->leftMotor.wheelSpeeds.lowSpeed);
  Serial.print(", ");
  Serial.print(robot->rightMotor.wheelSpeeds.highSpeed);
  Serial.print(", ");
  Serial.println(robot->rightMotor.wheelSpeeds.lowSpeed);

  data->speed += data->speedStep;
  if (data->speed > CALIBRATION_SPEED_LIMIT) {
    Serial.println("Speed calibration test finished.");
    setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    setParameter(PARAM_CALIBRATION_MODE, CALIBRATION_OFF);
    clearTestCalibrationData(data);
    return;
  }
  setParameter(PARAM_ROBOT_SPEED, data->speed);
}

void clearTestCalibrationData(TestCalibrationData* data) {
  data->speed = -CALIBRATION_SPEED_LIMIT;
}