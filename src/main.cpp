#include <Wire.h>

#include <globalConfig.h>
#include <utilities/params.h>

#include "./tasks/taskBattery.h"
#include "./tasks/taskButton.h"
#include "./tasks/taskBuzzer.h"
#include "./tasks/taskDcMotor.h"
#include "./tasks/taskEncoders.h"

SemaphoreHandle_t xSemaphoreWire = xSemaphoreCreateBinary();

// functions prototypes
void taskBlink();
void taskSerial();
void taskWifi();
void taskWebserver();
void taskWire();
void taskGY521();
void taskVL53L1X();
void taskEventSourceSender();
void taskDcMotorTest();

void setup() {
  // start serial communication
  Serial.begin(SERIAL_SPEED);  // only for debug purpose

  // start I2C communication
  xSemaphoreGive(xSemaphoreWire);
  // Wire.begin(SDA, SCL);
  // Wire.setClock(I2C_SPEED);

  // set default serial parameters values in case of reboot
  // todo: this doesn't work -> setAndSave? doesn't work either
  setAndSaveParameter(PARAM_DEBUG, DEBUG_BUTTON);

  setupParameters();
  taskSerial();
  taskWifi();
  taskWebserver();
  taskWire();
  // taskGY521();
  // taskVL53L1X();
  taskDcMotor();
  // taskBuzzer();
  taskEncodersX4();
  taskBattery();
  // taskButton();
  taskEventSourceSender();
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