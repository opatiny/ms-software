/**
 * Thread to handle the I2C communication with the five VL53L1X distance
 * sensors.
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L1X.h>
#include <Wire.h>

#include <utilities/params.h>

#define TIMING_BUDGET 140  // ms

void TaskVL53L1X(void* pvParameters) {
  vTaskDelay(1000);
  int addresses[] = {VL53_LEFT_ADDRESS, VL53_FRONT_LEFT_ADDRESS,
                     VL53_FRONT_ADDRESS, VL53_FRONT_RIGHT_ADDRESS,
                     VL53_RIGHT_ADDRESS};

  int parameters[] = {PARAM_DISTANCE_LEFT, PARAM_DISTANCE_FRONT_LEFT,
                      PARAM_DISTANCE_FRONT, PARAM_DISTANCE_FRONT_RIGHT,
                      PARAM_DISTANCE_RIGHT};

  Adafruit_VL53L1X sensors[5];

  // Create a VL53L1X sensor object
  Wire.begin(SDA, SCL);

  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    sensors[i] = Adafruit_VL53L1X();
  }

  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    if (!sensors[i].begin(addresses[i], &Wire)) {
      Serial.print("Error on init of distance sensor ");
      Serial.print(i);
      Serial.println(sensors[i].vl_status);
      while (1)
        delay(10);
    }
    if (PARAM_DEBUG) {
      Serial.print("Distance sensor ");
      Serial.print(i);
      Serial.println(" OK!");
    }
    if (!sensors[i].startRanging()) {
      Serial.print(F("Couldn't start ranging on sensor "));
      Serial.println(i);
      Serial.println(sensors[i].vl_status);
      while (1)
        delay(10);
    }
    sensors[i].setTimingBudget(TIMING_BUDGET);
  }

  int16_t distance;
  while (true) {
    vTaskDelay(50);
    for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
      if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
        distance = sensors[i].distance();  // read distance in mm
        xSemaphoreGive(xSemaphoreWire);
        if (distance == -1) {
          // something went wrong!
          Serial.print(F("Couldn't get distance on sensor "));
          Serial.print(i);
          Serial.println(sensors[i].vl_status);
          return;
        }
        setParameter(parameters[i], distance);
        if (getParameter(PARAM_DEBUG) == 1) {
          Serial.print(F("Distance"));
          Serial.print(i);
          Serial.print(F(": "));
          Serial.print(distance);
          Serial.println(" mm");
        }
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