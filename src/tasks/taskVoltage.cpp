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
#include "./taskVoltage.h"

#define DELAY 1000

void initializeVoltage(VoltageMeasurement* voltageMesurement);

void TaskVoltage(void* pvParameters) {
  uint32_t time = millis();  // in ms

  initializeVoltage(&robot.battery);
  initializeVoltage(&robot.vcc);

  while (true) {
    int batteryLevel = analogReadMilliVolts(BATTERY_PIN) *
                       (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    int vccLevel = analogReadMilliVolts(VCC_PIN) * (VCC_R1 + VCC_R2) / VCC_R2;
    // to access this debug mode: U4
    if (getParameter(PARAM_DEBUG) == DEBUG_BATTERY_LOG_DATA) {
      Serial.print(time);
      Serial.print(", \t");
      time += DELAY;
      Serial.print(batteryLevel);
      Serial.print(", \t");
      Serial.print(vccLevel);
      Serial.print(", \t");
      Serial.print(getParameter(PARAM_MOTOR_LEFT_MODE));
      Serial.print(", \t");
      Serial.println(getParameter(PARAM_MOTOR_RIGHT_MODE));
    }

    if (getParameter(PARAM_DEBUG) == DEBUG_BATTERY) {
      Serial.print("Battery voltage: ");
      Serial.print(batteryLevel / 1000.0, 2);
      Serial.print(" V | Vcc voltage: ");
      Serial.print(vccLevel / 1000.0, 2);
      Serial.println(" V");
    }
    if (getParameter(PARAM_BATTERY) == DEBUG_BATTERY) {
      if (batteryLevel <= BATTERY_EMPTY) {
        Serial.println("Warning: battery voltage is too low!");
      }
      if (getParameter(PARAM_VCC_VOLTAGE) == DEBUG_BATTERY) {
        if (vccLevel <= VCC_WARNING) {
          Serial.println("Warning: Vcc voltage is too low!");
        }
      }
    }
    setParameter(PARAM_BATTERY, batteryLevel);
    vTaskDelay(DELAY);
  }
}

void taskVoltage() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskVoltage, "TaskVoltage",
                          2048,  // Crashes if less than 1024 !!!!
                                 // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}

void initializeVoltage(VoltageMeasurement* voltageMeasurement) {
  voltageMeasurement->pin = BATTERY_PIN;
  voltageMeasurement->voltageParameter = PARAM_BATTERY;
  voltageMeasurement->warningVoltage = BATTERY_EMPTY;

  pinMode(voltageMeasurement->pin, INPUT);
}