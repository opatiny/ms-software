/**
 * Task to measure battery voltage.
 *
 * Debug: U3
 * Log data for matlab: U4
 */

#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>
#include "../pinMapping.h"
#include "../state.h"
#include "./taskBattery.h"
#include "./taskDcMotor.h"

#define DELAY 1000

void initializeBattery(Battery* battery);

void TaskBattery(void* pvParameters) {
  uint32_t batteryTime = 0;  // in ms

  initializeBattery(&robot.battery);

  while (true) {
    Battery battery = robot.battery;
    int measuredVoltage = analogReadMilliVolts(BATTERY_PIN);
    int batteryLevel = measuredVoltage * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    // to access this debug mode: U4
    if (getParameter(PARAM_DEBUG) == DEBUG_BATTERY_LOG_DATA) {
      Serial.print(batteryTime);
      Serial.print(", \t");
      batteryTime += DELAY;
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
    if (batteryLevel <= BATTERY_EMPTY &&
        getParameter(PARAM_BATTERY) == DEBUG_BATTERY) {
      Serial.println("Warning: battery is empty!");
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

void initializeBattery(Battery* battery) {
  battery->pin = BATTERY_PIN;
  battery->voltageParameter = PARAM_BATTERY;
  battery->warningVoltage = BATTERY_EMPTY;

  pinMode(battery->pin, INPUT);
}