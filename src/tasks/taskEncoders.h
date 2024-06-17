#pragma once

#define LOW_SPEED_NB_COUNTS 2
#define LOW_SPEED_MAX_DELAY_PER_COUNT \
  5000  // us -> corresponds to approx 10rpm,
        // speeds below that are considered 0
        // see SPEED_ZERO_THRESHOLD

#define LOW_SPEED_MAX_DELAY \
  (LOW_SPEED_NB_COUNTS * LOW_SPEED_MAX_DELAY_PER_COUNT)

struct EncoderParams {
  int pin1;
  int pin2;
};
void taskEncodersX4();
