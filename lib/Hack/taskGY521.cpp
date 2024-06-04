/**
 * Thread to communicate with the accelerometer.
 *
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/8ada83e8a572f8f5f3fa828a6459efc6a0985f04/lib/hack/taskGY521.cpp
 *
 * Debug: U2
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "../customUtilities/state.h"
#include "./taskGY521.h"
#include "./utilities/params.h"

#define LOG_DELAY 250

void setImuData(ImuData* imuData,
                sensors_event_t* a,
                sensors_event_t* g,
                sensors_event_t* temp);
void printImuDebug(ImuData* imuData);

void TaskGY521(void* pvParameters) {
  vTaskDelay(1000);

  Adafruit_MPU6050 mpu;

  if (!mpu.begin(IMU_ADDRESS, &Wire)) {
    Serial.println("Failed to find MPU6050 chip, IMU task is disabled.");
    vTaskDelay(1000);
    while (true) {
      vTaskDelay(1000);
    }
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

    sensors_event_t a, g, temp;
    int previousMillis = millis();
    while (true) {
      vTaskDelay(5);
      if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
        mpu.getEvent(&a, &g, &temp);
        xSemaphoreGive(xSemaphoreWire);
        setImuData(&robot.imuData, &a, &g, &temp);
      }
      if (getParameter(PARAM_DEBUG) == DEBUG_IMU) {
        if (millis() - previousMillis > LOG_DELAY) {
          printImuDebug(&robot.imuData);
          previousMillis = millis();
        }
      }
    }
  }
}

void taskGY521() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskGY521, "TaskGY521",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          3,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void setImuData(ImuData* imuData,
                sensors_event_t* a,
                sensors_event_t* g,
                sensors_event_t* temp) {
  int factor = 100;

  int ax = a->acceleration.x * factor;
  int ay = a->acceleration.y * factor;
  int az = a->acceleration.z * factor;
  int rx = g->gyro.x * factor;
  int ry = g->gyro.y * factor;
  int rz = g->gyro.z * factor;
  int temperature = temp->temperature;

  imuData->acceleration.x = ax;
  imuData->acceleration.y = ay;
  imuData->acceleration.z = az;
  imuData->rotation.x = rx;
  imuData->rotation.y = ry;
  imuData->rotation.z = rz;
  imuData->temperature = temperature;

  setParameter(PARAM_ACCELERATION_X, ax);
  setParameter(PARAM_ACCELERATION_Y, ay);
  setParameter(PARAM_ACCELERATION_Z, az);
  setParameter(PARAM_ROTATION_X, rx);
  setParameter(PARAM_ROTATION_Y, ry);
  setParameter(PARAM_ROTATION_Z, rz);
}

void printImuDebug(ImuData* imuData) {
  Serial.print("Acceleration: ");
  Serial.print(imuData->acceleration.x);
  Serial.print(", ");
  Serial.print(imuData->acceleration.y);
  Serial.print(", ");
  Serial.print(imuData->acceleration.z);
  Serial.print(" | Rotation: ");
  Serial.print(imuData->rotation.x);
  Serial.print(", ");
  Serial.print(imuData->rotation.y);
  Serial.print(", ");
  Serial.println(imuData->rotation.z);
}