#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>
#include "./taskBattery.h"

#define DELAY 1000
#define LOG_BATTERY_DATA 1

uint32_t buzzerTime = 0;  // in ms

void TaskBattery(void* pvParameters) {
  pinMode(BATTERY_PIN, INPUT);
  while (true) {
    int measuredVoltage = analogReadMilliVolts(BATTERY_PIN);
    int batteryLevel = measuredVoltage * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    if (getParameter(PARAM_DEBUG) == DEBUG_BATTERY_LOG_DATA) {
      Serial.print(buzzerTime);
      Serial.print(", \t");
      buzzerTime += DELAY;
      Serial.print(batteryLevel);
      Serial.print(", \t");
      Serial.print(getParameter(PARAM_MOTOR_LEFT_MODE));
      Serial.print(", \t");
      Serial.println(getParameter(PARAM_MOTOR_RIGHT_MODE));
    }

    if (getParameter(PARAM_DEBUG) == DEBUG_BATTERY) {
      Serial.print("Battery voltage: ");
      Serial.print(batteryLevel / 1000.0, 2);
      Serial.println(" V");
    }
    setParameter(PARAM_BATTERY, batteryLevel);
    vTaskDelay(DELAY);
  }
}

void taskBattery() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskBattery, "TaskBattery",
                          2048,  // Crashes if less than 1024 !!!!
                                 // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}
