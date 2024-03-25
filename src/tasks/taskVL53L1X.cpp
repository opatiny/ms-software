/**
 * Thread to handle the I2C communication with the five VL53L1X distance
 * sensors.
 */

#include <Adafruit_Sensor.h>
#include <VL53L1X.h>

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskVl53L1X.h"

void initialiseVL53L1X(VL53L1X sensors[NB_DISTANCE_SENSORS],
                       int xshutPins[],
                       int addresses[]);

int addresses[] = {VL53_LEFT_ADDRESS, VL53_FRONT_LEFT_ADDRESS,
                   VL53_FRONT_ADDRESS, VL53_FRONT_RIGHT_ADDRESS,
                   VL53_RIGHT_ADDRESS};

int distances[] = {PARAM_DISTANCE_LEFT, PARAM_DISTANCE_FRONT_LEFT,
                   PARAM_DISTANCE_FRONT, PARAM_DISTANCE_FRONT_RIGHT,
                   PARAM_DISTANCE_RIGHT};

int xshutPins[] = {XSHUT_PIN_LEFT, XSHUT_PIN_FRONT_LEFT, XSHUT_PIN_FRONT,
                   XSHUT_PIN_FRONT_RIGHT, XSHUT_PIN_RIGHT};

VL53L1X sensors[NB_DISTANCE_SENSORS];

void TaskVL53L1X(void* pvParameters) {
  setParameter(PARAM_DEBUG, DEBUG_DISTANCE);
  Serial.print("Debug value: ");
  Serial.println(getParameter(PARAM_DEBUG));

  vTaskDelay(1000);

  initialiseVL53L1X(sensors, xshutPins, addresses);

  if (getParameter(PARAM_DEBUG) == DEBUG_DISTANCE) {
    Serial.println(F("VL53L1X sensors initialized"));
  }

  while (true) {
    vTaskDelay(1000);
  }

  /*   int16_t distance;
    while (true) {
      vTaskDelay(50);
      for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
        if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
          distance = sensors[i].read();  // read distance in mm
          xSemaphoreGive(xSemaphoreWire);

          setParameter(distances[i], distance);
          if (getParameter(PARAM_DEBUG) == DEBUG_DISTANCE) {
            Serial.print(F("Distance"));
            Serial.print(i);
            Serial.print(F(": "));
            Serial.print(distance);
            Serial.println(" mm");
          }
        }
      }
    } */
}

void taskVL53L1X() {
  xTaskCreatePinnedToCore(TaskVL53L1X, "TaskVL53L1X",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          3,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void initialiseVL53L1X(VL53L1X sensors[NB_DISTANCE_SENSORS],
                       int xshutPins[],
                       int addresses[]) {
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < NB_DISTANCE_SENSORS; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < NB_DISTANCE_SENSORS; i++) {
    pinMode(xshutPins[i], INPUT);
    vTaskDelay(10);
    Serial.print("Initial address: ");
    Serial.println(sensors[i].getAddress());

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
    }

    Serial.print("Desired new address: ");
    Serial.println(addresses[i]);

    if (sensors[i].getAddress() != addresses[i]) {
      sensors[i].setAddress(addresses[i]);
    }

    Serial.print("Actual new address: ");
    Serial.println(sensors[i].getAddress());

    // start ranging and wait for TIMING_BUDGET ms between measurements.
    sensors[i].startContinuous(TIMING_BUDGET);
  }
}