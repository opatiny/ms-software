#include <Wire.h>

#include <eeprom.h>
#include <globalConfig.h>
#include <utilities/params.h>

#include "./pinMapping.h"
#include "./state.h"
#include "./tasks/taskButton.h"
#include "./tasks/taskBuzzer.h"
#include "./tasks/taskCalibrateSpeed.h"
#include "./tasks/taskEncoders.h"
#include "./tasks/taskOdometry.h"
#include "./tasks/taskRgbLed.h"
#include "./tasks/taskRobotMove.h"
#include "./tasks/taskVL53L1X.h"
#include "./tasks/taskVoltage.h"

SemaphoreHandle_t xSemaphoreWire = xSemaphoreCreateBinary();

Robot robot;

// functions prototypes
void taskBlink();
void taskSerial();
void taskWifi();
void taskWebserver();
void taskGY521();
void taskEventSourceSender();
void taskDcMotorTest();

void debugTask(char const* taskName);

void setup() {
  // start serial communication
  Serial.begin(SERIAL_SPEED);  // only for debug purpose
  // start I2C communication
  xSemaphoreGive(xSemaphoreWire);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_SPEED);

  vTaskDelay(1000);  // wait for connections to open
  Serial.println("Device is up");

  // load data from EEPROM
  loadWheelsRegressions(&robot.leftMotor.regressions,
                        &robot.rightMotor.regressions);

  setupParameters();

  taskSerial();
  debugTask("TaskSerial");

  taskWifi();
  debugTask("TaskWifi");

  taskWebserver();
  debugTask("TaskWebserver");

  taskGY521();
  debugTask("TaskGY521");

  // todo: requestFrom() I2C error in there
  taskVL53L1X();
  debugTask("TaskVL53L1X");

  taskRobotMove();
  debugTask("TaskRobotMove");

  taskEncodersX4();
  debugTask("TaskEncodersX4");

  taskOdometry();
  debugTask("TaskOdometry");

  // taskCalibrateSpeed();
  // debugTask("TaskCalibration");

  taskVoltage();
  debugTask("TaskVoltage");

  taskButton();
  debugTask("TaskButton");

  // taskBuzzer();
  // debugTask("TaskBuzzer");

  taskRgbLed();
  debugTask("TaskRgbLed");

  taskEventSourceSender();
  debugTask("TaskEvent");

  taskBlink();
  debugTask("TaskBlink");

  if (getParameter(PARAM_SOUND) == SOUND_ON) {
    setParameter(PARAM_BUZZER_MODE, BUZZER_BOOT);
  }
  // setParameter(PARAM_DEBUG, DEBUG_PROCESSES);
}

void loop() {
  vTaskDelay(1000);
}

void resetParameters() {
  for (byte i = 0; i < MAX_PARAM; i++) {
    setAndSaveParameter(i, ERROR_VALUE);
  }
}
