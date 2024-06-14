#include <Arduino.h>

double getSeconds() {
  return micros() / 1000000.0;
}

uint32_t secToMicros(double sec) {
  return sec * 1000000;
}

double microsToSeconds(uint32_t micros) {
  return micros / 1000000.0;
}