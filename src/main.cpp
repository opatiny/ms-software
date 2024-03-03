#include "./tasks/globalConfig.h"
#include "./tasks/utilities/params.h"

SemaphoreHandle_t xSemaphoreWire = xSemaphoreCreateBinary();

// functions prototypes
void taskBlink();
void taskSerial();
void taskWifi();
void taskWebserver();
void taskWire();
void taskGY521();

void setup() {
  xSemaphoreGive(xSemaphoreWire);
  Serial.begin(115200);  // only for debug purpose
  setupParameters();
  taskSerial();
  taskWifi();
  taskWebserver();
  taskWire();
  taskGY521();
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