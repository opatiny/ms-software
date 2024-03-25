#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>
#include "./taskBattery.h"

void TaskBattery(void* pvParameters) {
  pinMode(BATTERY_PIN, INPUT);

  while (true) {
    int measuredVoltage = analogReadMilliVolts(BATTERY_PIN);
    int batteryLevel = measuredVoltage * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    if (PARAM_DEBUG == DEBUG_BATTERY) {
      Serial.print("Battery level: ");
      Serial.print(batteryLevel);
      Serial.println(" mV");
    }
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
