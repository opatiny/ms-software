#pragma once

// button debounce time in ms
#define DEBOUNCE_TIME 50

enum ButtonState { BUTTON_RELEASED, BUTTON_PRESSED };

struct ButtonFlags {
  bool distanceCalibration = false;
  bool robotMode = false;
};

extern ButtonFlags buttonFlags;

void taskButton();