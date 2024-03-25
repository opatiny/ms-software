/**
 * Thread to communicate with the accelerometer.
 *
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/8ada83e8a572f8f5f3fa828a6459efc6a0985f04/lib/hack/taskGY521.cpp
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "./utilities/params.h"

#define IMU_ADDRESS 0x68

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
    while (true) {
      vTaskDelay(5);
      if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
        mpu.getEvent(&a, &g, &temp);
        xSemaphoreGive(xSemaphoreWire);
        setParameter(PARAM_ACCELERATION_X, a.acceleration.x * 100);
        setParameter(PARAM_ACCELERATION_Y, a.acceleration.y * 100);
        setParameter(PARAM_ACCELERATION_Z, a.acceleration.z * 100);
        setParameter(PARAM_ROTATION_X, g.gyro.x * 100);
        setParameter(PARAM_ROTATION_Y, g.gyro.y * 100);
        setParameter(PARAM_ROTATION_Z, g.gyro.z * 100);
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