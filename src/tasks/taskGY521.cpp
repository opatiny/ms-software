#include "config.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "./utilities/params.h"

void TaskGY521(void* pvParameters) {
  vTaskDelay(1000);

  Wire.begin(SDA, SCL);

  Adafruit_MPU6050 mpu;

  if (!mpu.begin(IMU_ADDRESS, &Wire)) {
    Serial.println("Failed to find MPU6050 chip");
    vTaskDelay(1000);
  }

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