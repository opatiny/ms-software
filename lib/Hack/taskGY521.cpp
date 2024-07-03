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
                sensors_event_t* temp,
                ImuData* offsets);
void printImuDebug(ImuData* imuData);
ImuData getCalibrationOffsets(Adafruit_MPU6050* mpu);

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

    ImuData offsets = getCalibrationOffsets(&mpu);

    sensors_event_t a, g, temp;

    int previousMillis = millis();
    while (true) {
      debugProcess("TaskGY521 ");
      vTaskDelay(5);
      if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
        // linear accel in m/s^2 and angular velocity in rad/s
        mpu.getEvent(&a, &g, &temp);
        xSemaphoreGive(xSemaphoreWire);
        setImuData(&robot.imuData, &a, &g, &temp, &offsets);
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
                sensors_event_t* temp,
                ImuData* offsets) {
  int factor = 100;

  double ax = a->acceleration.x - offsets->acceleration.x;
  double ay = a->acceleration.y - offsets->acceleration.y;
  double az = a->acceleration.z - offsets->acceleration.z;
  double rx = g->gyro.x - offsets->rotation.x;
  double ry = g->gyro.y - offsets->rotation.y;
  double rz = g->gyro.z - offsets->rotation.z;
  double temperature = temp->temperature;

  imuData->acceleration.x = ax;
  imuData->acceleration.y = ay;
  imuData->acceleration.z = az;
  imuData->rotation.x = rx;
  imuData->rotation.y = ry;
  imuData->rotation.z = rz;
  imuData->temperature = temperature;

  // store serial params with the factor
  setParameter(PARAM_ACCELERATION_X, ax * factor);
  setParameter(PARAM_ACCELERATION_Y, ay * factor);
  setParameter(PARAM_ACCELERATION_Z, az * factor);
  setParameter(PARAM_ROTATION_X, rx * factor);
  setParameter(PARAM_ROTATION_Y, ry * factor);
  setParameter(PARAM_ROTATION_Z, rz * factor);
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

ImuData getCalibrationOffsets(Adafruit_MPU6050* mpu) {
  // we make an average on multiple measures
  const int nbMeasures = 10;
  // target values of the imu when the robot is not moving and on a flat surface
  const ImuData targets = {0, 0, -9.81, 0, 0, 0, 0};
  ImuData offsets;
  for (int i = 0; i < nbMeasures; i++) {
    sensors_event_t a, g, temp;
    if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
      // linear accel in m/s^2 and angular velocity in rad/s
      mpu->getEvent(&a, &g, &temp);
      xSemaphoreGive(xSemaphoreWire);
      offsets.acceleration.x += a.acceleration.x;
      offsets.acceleration.y += a.acceleration.y;
      offsets.acceleration.z += a.acceleration.z;
      offsets.rotation.x += g.gyro.x;
      offsets.rotation.y += g.gyro.y;
      offsets.rotation.z += g.gyro.z;
    }
    vTaskDelay(5);
  }

  offsets.acceleration.x /= nbMeasures;
  offsets.acceleration.y /= nbMeasures;
  offsets.acceleration.z /= nbMeasures;
  offsets.rotation.x /= nbMeasures;
  offsets.rotation.y /= nbMeasures;
  offsets.rotation.z /= nbMeasures;

  offsets.acceleration.x -= targets.acceleration.x;
  offsets.acceleration.y -= targets.acceleration.y;
  offsets.acceleration.z -= targets.acceleration.z;
  offsets.rotation.x -= targets.rotation.x;
  offsets.rotation.y -= targets.rotation.y;
  offsets.rotation.z -= targets.rotation.z;

  return offsets;
}