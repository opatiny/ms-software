/**
 * Task to measure battery voltage.
 *
 * Debug: U3
 * Log data for matlab: U4
 */

#include <Arduino.h>

#include <globalConfig.h>
#include <pinMapping.h>
#include <state.h>
#include <timeUtilities.h>
#include <utilities/params.h>

#include "taskRgbLed.h"
#include "taskVoltage.h"

#define DELAY 1000           // ms
#define WARNING_DELAY 10000  // ms

void initializeVoltage(VoltageMeasurement* voltageMeasurement,
                       VoltageMeasurement* params);

void TaskVoltage(void* pvParameters) {
  int previousBatteryState = 1;  // 0: empty, 1: full
  uint32_t time = millis();      // in ms

  VoltageMeasurement battery = {
    voltageParameter : PARAM_BATTERY_VOLTAGE,
    pin : BATTERY_PIN,
    warningVoltage : BATTERY_EMPTY,
  };
  VoltageMeasurement vcc = {
    voltageParameter : PARAM_VCC_VOLTAGE,
    pin : VCC_PIN,
    warningVoltage : VCC_WARNING,
  };

  initializeVoltage(&robot.battery, &battery);
  initializeVoltage(&robot.vcc, &vcc);

  while (true) {
    int batteryLevel = analogReadMilliVolts(BATTERY_PIN) *
                       (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
    int vccLevel = analogReadMilliVolts(VCC_PIN) * (VCC_R1 + VCC_R2) / VCC_R2;
    // to access this debug mode: U4
    if (getParameter(PARAM_DEBUG) == DEBUG_BATTERY_LOG_DATA) {
      Serial.print(getSeconds(), 3);
      Serial.print(", \t");
      Serial.print(batteryLevel);
      Serial.print(", \t");
      Serial.print(vccLevel);
      Serial.print(", \t");
      Serial.print(getParameter(PARAM_MOTOR_LEFT_MODE));
      Serial.print(", \t");
      Serial.println(getParameter(PARAM_MOTOR_RIGHT_MODE));
    }

    if (getParameter(PARAM_DEBUG) == DEBUG_VOLTAGES) {
      Serial.print("Battery voltage: ");
      Serial.print(batteryLevel / 1000.0, 2);
      Serial.print(" V | Vcc voltage: ");
      Serial.print(vccLevel / 1000.0, 2);
      Serial.println(" V");
    }

    // blink RGB LED if battery is empty
    if (batteryLevel <= BATTERY_EMPTY) {
      if (previousBatteryState == 1) {
        setParameter(PARAM_RGB_LED_MODE, LED_BLINK);
        previousBatteryState = 0;
      }
    } else {
      if (previousBatteryState == 0) {
        setParameter(PARAM_RGB_LED_MODE, LED_OFF);
        previousBatteryState = 1;
      }
    }

    if (millis() - time > WARNING_DELAY) {
      if (batteryLevel <= BATTERY_EMPTY) {
        Serial.println("Warning: battery is empty!");
      }
      if (getParameter(PARAM_VCC_VOLTAGE) == DEBUG_VOLTAGES) {
        if (vccLevel <= VCC_WARNING) {
          Serial.println("Warning: Vcc voltage is too low!");
        }
      }
      time = millis();
    }
    setParameter(PARAM_BATTERY_VOLTAGE, batteryLevel);
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

void initializeVoltage(VoltageMeasurement* voltageMeasurement,
                       VoltageMeasurement* params) {
  voltageMeasurement->pin = params->pin;
  voltageMeasurement->voltageParameter = params->voltageParameter;
  voltageMeasurement->warningVoltage = params->warningVoltage;

  pinMode(voltageMeasurement->pin, INPUT);
}