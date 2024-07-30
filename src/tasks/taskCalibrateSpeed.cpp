/**
 * Task for the speed calibration of the wheels. Allows to find the parameters
 * of the polynomial regressions that fit the command VS speed curve.
 */

#include <globalConfig.h>
#include <kinematics.h>
#include <motorProperties.h>
#include <printUtilities.h>
#include <robotModes.h>
#include <speedCalibration.h>
#include <state.h>
#include <utilities/params.h>

#include "taskRobotMove.h"

void TaskCalibration(void* pvParameters) {
  CalibrationData calibrationData;
  initialiseCalibrationData(&calibrationData);

  TestCalibrationData testData;
  initialiseTestCalibrationData(&testData);

  while (true) {
    switch (getParameter(PARAM_CALIBRATION_MODE)) {
      case CALIBRATION_ON:
        wheelSpeedCalibration(&calibrationData, &robot.leftMotor.regressions,
                              &robot.rightMotor.regressions);
        break;
      case CALIBRATION_TEST:
        testCalibration(&robot, &testData);
        break;
      case CALIBRATION_OFF:
        break;
      default:
        break;
    }

    // handle case where user changes debug mode before calibration is finished
    if (getParameter(PARAM_CALIBRATION_MODE) == CALIBRATION_OFF &&
        calibrationData.command != MIN_MOTOR_COMMAND) {
      Serial.println("Warning: Speed calibration stopped before finishing.");
      clearCalibrationData(&calibrationData);
      setParameter(PARAM_ROBOT_COMMAND, 0);
      setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    }

    if (getParameter(PARAM_CALIBRATION_MODE) == CALIBRATION_OFF &&
        testData.speed != -CALIBRATION_SPEED_LIMIT) {
      clearTestCalibrationData(&testData);
      setParameter(PARAM_ROBOT_WHEELS_SPEED, 0);
      setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    }
    vTaskDelay(SPEED_CALIBRATION_DELAY);
  }
}

void taskCalibrateSpeed() {
  xTaskCreatePinnedToCore(TaskCalibration, "TaskCalibration", 131072, NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // on core 1
}
