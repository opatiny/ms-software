/**
 * Thread to handle the I2C communication with the five VL53L1X distance
 * sensors. The distances are from the center of the robot!!
 * Set the distance debug mode with parameter AH.
 * Debug: U1
 */

#include <Adafruit_Sensor.h>
#include <VL53L1X.h>

#include <globalConfig.h>
#include <pinMapping.h>
#include <state.h>
#include <timeUtilities.h>
#include <utilities/params.h>

#include "./taskButton.h"
#include "./taskVl53L1X.h"

#define DISTANCE_TASK_DELAY 50

void initialiseVL53L1X(VL53L1X sensors[NB_DISTANCE_SENSORS],
                       int xshutPins[],
                       int addresses[]);

void distanceSensorsDebug(int* distancesParameters);

int addresses[] = {VL53_LEFT_ADDRESS, VL53_FRONT_LEFT_ADDRESS,
                   VL53_FRONT_ADDRESS, VL53_FRONT_RIGHT_ADDRESS,
                   VL53_RIGHT_ADDRESS};

int distancesParameters[] = {PARAM_DISTANCE_LEFT, PARAM_DISTANCE_FRONT_LEFT,
                             PARAM_DISTANCE_FRONT, PARAM_DISTANCE_FRONT_RIGHT,
                             PARAM_DISTANCE_RIGHT};

int xshutPins[] = {XSHUT_PIN_LEFT, XSHUT_PIN_FRONT_LEFT, XSHUT_PIN_FRONT,
                   XSHUT_PIN_FRONT_RIGHT, XSHUT_PIN_RIGHT};

/**
 * Distances from the center of the robot to the sensors in mm.
 */
const int offsets[] = {22, 54, 50, 54, 22};

void TaskVL53L1X(void* pvParameters) {
  VL53L1X sensors[NB_DISTANCE_SENSORS];
  vTaskDelay(2000);
  initialiseVL53L1X(sensors, xshutPins, addresses);
  vTaskDelay(1000);

  int16_t distance;
  while (true) {
    debugProcess("TaskVL53L1X ");
    vTaskDelay(DISTANCE_TASK_DELAY);
    for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
      if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
        distance = sensors[i].read();  // read distance in mm
        xSemaphoreGive(xSemaphoreWire);

        const int correctedDistance = distance + offsets[i];

        robot.distances[i] = correctedDistance;
        setParameter(distancesParameters[i], correctedDistance);
      }
    }
    distanceSensorsDebug(distancesParameters);
  }
}

void taskVL53L1X() {
  xTaskCreatePinnedToCore(TaskVL53L1X, "TaskVL53L1X",
                          8192,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          3,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

/**
 * Initialize all VL53L1X sensors by changing their I2C addresses.
 */
void initialiseVL53L1X(VL53L1X sensors[NB_DISTANCE_SENSORS],
                       int xshutPins[],
                       int addresses[]) {
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < NB_DISTANCE_SENSORS; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  if (getParameter(PARAM_DEBUG) == DEBUG_DISTANCE) {
    Serial.println(F("Initializing VL53L1X sensors ..."));
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < NB_DISTANCE_SENSORS; i++) {
    pinMode(xshutPins[i], INPUT);
    vTaskDelay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
    }

    if (addresses[i] == VL53_DEFAULT_ADDRESS) {
      Serial.println("New address should be different from the default one");
    }

    sensors[i].setAddress(addresses[i]);

    if (getParameter(PARAM_DEBUG) == DEBUG_DISTANCE) {
      Serial.print("New distance sensor ");
      Serial.print(i);
      Serial.print(" address: ");
      Serial.println(sensors[i].getAddress());
    }
    // start ranging and wait for TIMING_BUDGET ms between measurements.
    sensors[i].startContinuous(TIMING_BUDGET);
  }
}

void distanceSensorsDebug(int* distancesParameters) {
  if (getParameter(PARAM_DEBUG) == DEBUG_DISTANCE) {
    switch (getParameter(PARAM_DISTANCE_DEBUG_MODE)) {
      case CONSTANT:
        Serial.print(getSeconds(), 3);
        for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
          Serial.print(F(",\t"));
          Serial.print(getParameter(distancesParameters[i]));
        }
        Serial.println();
        break;
      case CALIBRATION:
        if (buttonFlags.distanceCalibration) {
          for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
            Serial.print(getParameter(distancesParameters[i]));
            Serial.print(F(",\t"));
          }
          Serial.println();
          buttonFlags.distanceCalibration = false;
        }
        break;
      case RANGING:
      default:
        for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
          Serial.print(i);
          Serial.print(F(": "));
          Serial.print(getParameter(distancesParameters[i]));
          Serial.print(F("\t"));
        }
        Serial.println();
        break;
    }
  }
}