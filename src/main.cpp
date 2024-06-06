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

void setup() {
  // start serial communication
  Serial.begin(SERIAL_SPEED);  // only for debug purpose
  delay(2000);                 // wait for serial connection to open
  Serial.println("Device is up");

  // start I2C communication
  xSemaphoreGive(xSemaphoreWire);

  Wire.setClock(I2C_SPEED);
  Wire.begin(SDA_PIN, SCL_PIN);

  // set default serial parameters values in case of reboot
  // todo: this doesn't work -> setAndSave? doesn't work either
  setAndSaveParameter(PARAM_DEBUG, DEBUG_BUTTON);

  // load data from EEPROM
  loadWheelsRegressions(&robot.leftMotor.regressions,
                        &robot.rightMotor.regressions);

  setupParameters();
  taskSerial();
  // taskWifi();
  // taskWebserver();
  taskWire();
  // taskGY521();
  // taskVL53L1X();
  taskRobotMove();
  taskEncodersX4();
  taskOdometry();
  taskCalibrateSpeed();
  taskVoltage();
  // taskButton();
  // taskBuzzer();
  taskRgbLed();
  // taskEventSourceSender();
  taskBlink();
}

void loop() {
  vTaskDelay(1000);
}

void resetParameters() {
  for (byte i = 0; i < MAX_PARAM; i++) {
    setAndSaveParameter(i, ERROR_VALUE);
  }
}