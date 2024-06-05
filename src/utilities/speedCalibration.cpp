#include <globalConfig.h>
#include <kinematics.h>
#include <state.h>
#include <utilities/params.h>

#include "../tasks/taskRobotMove.h"
#include "printUtilities.h"

#include "speedCalibration.h"

void initialiseCalibrationData(CalibrationData* data) {
  data->command = MIN_MOTOR_COMMAND;
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
  int currentTime = millis();
  if (data->command == MIN_MOTOR_COMMAND) {
    Serial.println("Speed calibration started...\n");
    setParameter(PARAM_ROBOT_MODE, ROBOT_MOVE);
  }
  Serial.print(currentTime);
  Serial.print(", ");
  Serial.print(getParameter(PARAM_BATTERY_VOLTAGE));
  Serial.print(", ");
  Serial.print(data->command);
  Serial.print(", ");
  Serial.print(robot.odometry.leftWheelSpeed);
  Serial.print(", ");
  Serial.println(robot.odometry.rightWheelSpeed);

  data->commands[data->index] = data->command;
  data->leftSpeeds[data->index] = robot.odometry.leftWheelSpeed;
  data->rightSpeeds[data->index] = robot.odometry.rightWheelSpeed;

  data->command += COMMAND_STEP;
  // end condition for the calibration
  if (data->command > MAX_MOTOR_COMMAND) {
    // find the regressions
    double* x = data->commands;
    double* y = data->leftSpeeds;
    getRegressions(leftRegressions, x, y, CALIBRATION_SPEED_LIMIT);
    x = data->commands;
    y = data->rightSpeeds;
    getRegressions(rightRegressions, x, y, CALIBRATION_SPEED_LIMIT);

    printRegressionsForMatlab(leftRegressions, 10);
    Serial.println("");
    printRegressionsForMatlab(rightRegressions, 10);

    Serial.println("\nLeft wheel regressions:");
    printRegressions(leftRegressions, 5);
    Serial.println("Right wheel regressions:");
    printRegressions(rightRegressions, 5);

    Serial.println("Speed calibration finished.");
    setParameter(PARAM_CALIBRATE_SPEED, CALIBRATION_OFF);
    clearCalibrationData(data);
    setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    setParameter(PARAM_ROBOT_SPEED_CMD, 0);
    return;
  }
  setParameter(PARAM_ROBOT_SPEED_CMD, data->command);
  data->index++;
}

void clearCalibrationData(CalibrationData* data) {
  for (int i = 0; i < CALIBRATION_MAX_NB_VALUES; i++) {
    data->commands[i] = 0;
    data->leftSpeeds[i] = 0;
    data->rightSpeeds[i] = 0;
  }
  data->index = 0;
  data->command = MIN_MOTOR_COMMAND;
}
