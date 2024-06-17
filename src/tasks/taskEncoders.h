#pragma once

#define LOW_SPEED_NB_COUNTS 2

struct EncoderParams {
  int pin1;
  int pin2;
};
void taskEncodersX4();
