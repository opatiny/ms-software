/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/toHex.cpp
 */

#include "../globalConfig.h"
/* Functions to convert a number to hexadecimal */

const char hex[] = "0123456789ABCDEF";

uint8_t toHex(Print* output, byte value) {
  output->print(hex[value >> 4 & 15]);
  output->print(hex[value >> 0 & 15]);
  return value;
}

uint8_t toHex(Print* output, int16_t value) {
  byte checkDigit = toHex(output, (byte)(value >> 8 & 255));
  checkDigit ^= toHex(output, (byte)(value >> 0 & 255));
  return checkDigit;
}

uint8_t toHex(Print* output, long value) {
  byte checkDigit = toHex(output, (int16_t)(value >> 16 & 65535));
  checkDigit ^= toHex(output, (int16_t)(value >> 0 & 65535));
  return checkDigit;
}

uint8_t toHex(Print* output, unsigned long value) {
  byte checkDigit = toHex(output, (int16_t)(value >> 16 & 65535));
  checkDigit ^= toHex(output, (int16_t)(value >> 0 & 65535));
  return checkDigit;
}
