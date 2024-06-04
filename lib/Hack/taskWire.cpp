/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/58935a0e08a7047737f2120dbdf74b15b08bec4d/lib/hack/taskWire.cpp
 */

#include <Wire.h>

#include "../customUtilities/state.h"
#include "./globalConfig.h"
#include "./taskWire.h"
#include "./utilities/params.h"

#define WIRE_MAX_DEVICES 8
byte numberI2CDevices = 0;
byte wireDeviceID[WIRE_MAX_DEVICES];

void wireUpdateList(int8_t sleepTime);

void TaskWire(void* pvParameters) {
  vTaskDelay(100);
  (void)pvParameters;

  Wire.begin(SDA_PIN, SCL_PIN);

  while (true) {
    wireUpdateList();
    vTaskDelay(1000);
  }
}

void taskWire() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskWire, "TaskWire",
                          4048,  // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

int wireReadInt(uint8_t address) {
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() != 2) {
    return ERROR_VALUE;
  }
  int16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}

void wireWakeup(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.endTransmission();  // Send data to I2C dev with option for a repeated
                           // start
}

void wireSetRegister(uint8_t address, uint8_t registerAddress) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.endTransmission();  // Send data to I2C dev with option for a repeated
                           // start
}

int wireReadIntRegister(uint8_t address, uint8_t registerAddress) {
  wireSetRegister(address, registerAddress);
  return wireReadInt(address);
}

void wireCopyParameter(uint8_t address,
                       uint8_t registerAddress,
                       uint8_t parameterID) {
  setParameter(parameterID, wireReadIntRegister(address, registerAddress));
}

void wireWriteIntRegister(uint8_t address, uint8_t registerAddress, int value) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  if (value > 255 || value < 0)
    Wire.write(value >> 8);
  Wire.write(value & 255);
  Wire.endTransmission();  // Send data to I2C dev with option for a repeated
                           // start
}

void printWireInfo(Print* output) {
  wireUpdateList(1);
  output->println("I2C");

  for (byte i = 0; i < numberI2CDevices; i++) {
    output->print(i);
    output->print(F(": "));
    output->print(wireDeviceID[i]);
    output->print(F(" - "));
    output->println(wireDeviceID[i], BIN);
  }
}

void printWireDeviceParameter(Print* output, uint8_t wireID) {
  output->println(F("I2C device: "));
  output->println(wireID);
  for (byte i = 0; i < 26; i++) {
    output->print((char)(i + 65));
    output->print(" : ");
    output->print(i);
    output->print(F(" - "));
    output->println(wireReadIntRegister(wireID, i));
  }
}

void wireRemoveDevice(byte id) {
  for (byte i = id; i < numberI2CDevices - 1; i++) {
    wireDeviceID[i] = wireDeviceID[i + 1];
  }
  numberI2CDevices--;
}

void wireInsertDevice(byte id, byte newDevice) {
  // Serial.println(id);

  if (numberI2CDevices < WIRE_MAX_DEVICES) {
    for (byte i = id + 1; i < numberI2CDevices - 1; i++) {
      wireDeviceID[i] = wireDeviceID[i + 1];
    }
    wireDeviceID[id] = newDevice;
    numberI2CDevices++;
  }
}

boolean wireDeviceExists(byte id) {
  for (byte i = 0; i < numberI2CDevices; i++) {
    if (wireDeviceID[i] == id)
      return true;
  }
  return false;
}

void wireUpdateList(int8_t sleepTime) {
  // 16ms
  byte _data;
  byte currentPosition = 0;
  // I2C Module Scan, from_id ... to_id
  for (byte i = 0; i <= 127; i++) {
    if (xSemaphoreTake(xSemaphoreWire, 1) == pdTRUE) {
      Wire.beginTransmission(i);
      Wire.write(&_data, 0);
      // I2C Module found out!
      if (Wire.endTransmission() == 0) {
        // there is a device, we need to check if we should add or remove a
        // previous device
        if (currentPosition < numberI2CDevices &&
            wireDeviceID[currentPosition] ==
                i) {  // it is still the same device that is at the same
                      // position, nothing to do
          currentPosition++;
        } else if (currentPosition < numberI2CDevices &&
                   wireDeviceID[currentPosition] <
                       i) {  // some device(s) disappear, we need to delete them
          wireRemoveDevice(currentPosition);
          i--;
        } else if (currentPosition >= numberI2CDevices ||
                   wireDeviceID[currentPosition] >
                       i) {  // we need to add a device
          wireInsertDevice(currentPosition, i);
          currentPosition++;
        }
      }
      xSemaphoreGive(xSemaphoreWire);
    }
    vTaskDelay(
        sleepTime);  // not too fast because we need time for other processes
  }
  while (currentPosition < numberI2CDevices) {
    wireRemoveDevice(currentPosition);
  }
}

void wireUpdateList() {
  wireUpdateList(50);
}

void printWireHelp(Print* output) {
  output->println(F("(il) List devices"));
  output->println(F("(ip) List parameters"));
}

void processWireCommand(char command,
                        char* paramValue,
                        Print* output) {  // char and char* ??
  switch (command) {
    case 'p':
      if (paramValue[0] == '\0') {
        output->println(F("Missing device ID"));
      } else {
        printWireDeviceParameter(output, atoi(paramValue));
      }
      break;
    case 'l':
      printWireInfo(output);
      break;
    default:
      printWireHelp(output);
  }
}