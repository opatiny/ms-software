#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>
#include "./taskBattery.h"

void TaskBattery(void* pvParameters) {
  (void)pvParameters;

  pinMode(BATTERY_PIN, INPUT);

  while (true) {
    int measurementVoltage = analogReadMilliVolts(BATTERY_PIN);
    int batteryLevel = VBAT(measurementVoltage);
    setParameter(PARAM_BATTERY, batteryLevel);
    vTaskDelay(1000);
  }
}

void taskBattery() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskBattery, "TaskBattery",
                          1024,  // Crashes if less than 1024 !!!!
                                 // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          1,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}
