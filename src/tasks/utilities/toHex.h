/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/toHex.h
 */
#pragma once

uint8_t toHex(Print* output, byte value);

uint8_t toHex(Print* output, int16_t value);

uint8_t toHex(Print* output, long value);

uint8_t toHex(Print* output, unsigned long value);