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
void taskWire();
void taskGY521();
void taskEventSourceSender();
void taskDcMotorTest();

void debugTask(char const* taskName);

void setup() {
  // start serial communication
  Serial.begin(SERIAL_SPEED);  // only for debug purpose
  delay(500);                  // wait for serial connection to open
  Serial.println("Device is up");

  // start I2C communication
  xSemaphoreGive(xSemaphoreWire);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_SPEED);

  // set default serial parameters values in case of reboot
  // todo: this doesn't work -> setAndSave? doesn't work either
  setAndSaveParameter(PARAM_DEBUG, DEBUG_BUTTON);

  // load data from EEPROM
  loadWheelsRegressions(&robot.leftMotor.regressions,
                        &robot.rightMotor.regressions);

  setupParameters();

  taskSerial();
  debugTask("TaskSerial");

  // taskWifi();
  // debugTask("TaskWifi");

  // taskWebserver();
  // debugTask("TaskWebserver");

  taskWire();  // stack size problem?
  debugTask("TaskWire");

  // taskGY521();
  // debugTask("TaskGY521");

  taskVL53L1X();
  debugTask("TaskVL53L1X");

  taskRobotMove();
  debugTask("TaskRobotMove");

  taskEncodersX4();
  debugTask("TaskEncodersX4");

  taskOdometry();
  debugTask("TaskOdometry");

  taskCalibrateSpeed();
  debugTask("TaskCalibration");

  taskVoltage();
  debugTask("TaskVoltage");

  // taskButton();
  // debugTask("TaskButton");

  // taskBuzzer();
  // debugTask("TaskBuzzer");

  // taskRgbLed();
  // debugTask("TaskRgbLed");

  // taskEventSourceSender();

  taskBlink();
  debugTask("TaskBlink");
}

void loop() {
  vTaskDelay(1000);
}

void resetParameters() {
  for (byte i = 0; i < MAX_PARAM; i++) {
    setAndSaveParameter(i, ERROR_VALUE);
  }
}

void debugTask(char const* taskName) {
  if (getParameter(PARAM_DEBUG) == DEBUG_MAIN) {
    Serial.print("Task ");
    Serial.print(taskName);
    Serial.print(" running, remaining stack: ");
    TaskHandle_t handle = xTaskGetHandle(taskName);
    Serial.println(uxTaskGetStackHighWaterMark(handle));
  }
}