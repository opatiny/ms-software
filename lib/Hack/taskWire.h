/**
 * Thread allowing I2C communication with the follower devices on the bus of the
 * microcontroller.
 *
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/taskWire.h
 */

#include <Arduino.h>

int wireReadInt(uint8_t address);

void wireWakeup(uint8_t address);

void wireSetRegister(uint8_t address, uint8_t registerAddress);

int wireReadIntRegister(uint8_t address, uint8_t registerAddress);

void wireCopyParameter(uint8_t address,
                       uint8_t registerAddress,
                       uint8_t parameterID);

void wireWriteIntRegister(uint8_t address, uint8_t registerAddress, int value);

void printWireInfo(Print* output);

void printWireDeviceParameter(Print* output, uint8_t wireID);

void wireRemoveDevice(byte id);

void wireInsertDevice(byte id, byte newDevice);

boolean wireDeviceExists(byte id);

void wireUpdateList();

void printWireHelp(Print* output);

void processWireCommand(char command, char* paramValue, Print* output);
