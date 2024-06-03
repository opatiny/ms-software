#include <globalConfig.h>
#include <kinematics.h>
#include <utilities/params.h>

#include "../state.h"
#include "../tasks/taskRobotMove.h"

#include "speedCalibration.h"

/**
 * Non-blocking calibration function (can be stopped at any time).
 * Allows to find the polynomial relationship between the motor command and the
 * wheel speed for both wheels
 */
void wheelSpeedCalibration(int* command, CalibrationData* data) {
  int currentTime = millis();
  if (currentTime - data->previousTime > SPEED_CALIBRATION_DELAY) {
    if (*command == MIN_MOTOR_COMMAND) {
      Serial.println("Speed calibration started...\n");
      setParameter(PARAM_ROBOT_MODE, ROBOT_MOVE);
    }
    Serial.print(currentTime);
    Serial.print(", ");
    Serial.print(getParameter(PARAM_BATTERY_VOLTAGE));
    Serial.print(", ");
    Serial.print(*command);
    Serial.print(", ");
    Serial.print(robot.odometry.leftWheelSpeed);
    Serial.print(", ");
    Serial.println(robot.odometry.rightWheelSpeed);

    data->commands[data->index] = *command;
    data->leftSpeeds[data->index] = robot.odometry.leftWheelSpeed;
    data->rightSpeeds[data->index] = robot.odometry.rightWheelSpeed;

    *command += COMMAND_STEP;
    if (*command > MAX_MOTOR_COMMAND) {
      Serial.println("Speed calibration finished.");
      setParameter(PARAM_DEBUG, NO_DEBUG);
      *command = -MIN_MOTOR_COMMAND;
      setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
      setParameter(PARAM_ROBOT_SPEED_CMD, 0);

      data->index = 0;
      return;
    }
    setParameter(PARAM_ROBOT_SPEED_CMD, *command);
    data->previousTime = currentTime;
    data->index++;
  }
}
