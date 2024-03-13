/**
 * Thread to communicate with one of the distance sensor.
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L1X.h>
#include <Wire.h>

#include "utilities/params.h"

void TaskVL53L1X(void* pvParameters) {
  vTaskDelay(1000);

#define IRQ_PIN 2
#define XSHUT_PIN 3
  Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

  Wire.begin(SDA, SCL);

  if (!vl53.begin(VL53_FRONT_ADRESS, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)
      delay(10);
  }

  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)
      delay(10);
  }

  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  int16_t distance;
  while (true) {
    vTaskDelay(50);
    if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
      distance = vl53.distance();  // read distance in mm
      xSemaphoreGive(xSemaphoreWire);
      if (distance == -1) {
        // something went wrong!
        Serial.print(F("Couldn't get distance: "));
        Serial.println(vl53.vl_status);
        return;
      }
      setParameter(PARAM_DISTANCE_FRONT, distance);
      if (getParameter(PARAM_DEBUG) == 1) {
        Serial.print(F("Distance: "));
        Serial.print(distance);
        Serial.println(" mm");
      }
    }
  }
}

void taskVL53L1X() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskVL53L1X, "TaskVL53L1X",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          3,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}