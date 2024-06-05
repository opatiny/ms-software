#include <globalConfig.h>
#include <kinematics.h>
#include <state.h>
#include <utilities/params.h>
#include <utilities/printUtilities.h>
#include <utilities/speedCalibration.h>

#include "../hardwareProperties.h"
#include "taskRobotMove.h"

void TaskCalibrateSpeed(void* pvParameters) {
  CalibrationData calibrationData;
  initialiseCalibrationData(&calibrationData);

  while (true) {
    if (getParameter(PARAM_CALIBRATE_SPEED) == CALIBRATION_ON) {
      wheelSpeedCalibration(&calibrationData, &robot.leftMotor.regressions,
                            &robot.rightMotor.regressions);
    }

    // handle case where user changes debug mode before calibration is finished
    if (getParameter(PARAM_CALIBRATE_SPEED) == CALIBRATION_OFF &&
        calibrationData.command != MIN_MOTOR_COMMAND) {
      Serial.println("Warning: Speed calibration stopped before finishing.");
      clearCalibrationData(&calibrationData);
      setParameter(PARAM_ROBOT_SPEED_CMD, 0);
      setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    }
    vTaskDelay(SPEED_CALIBRATION_DELAY);
  }
}

void taskCalibrateSpeed() {
  xTaskCreatePinnedToCore(TaskCalibrateSpeed, "TaskCalibrateSpeed", 65536, NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 0);  // attached on core 0!!
}
