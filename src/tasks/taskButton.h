#pragma once

#define BUTTON_PIN D2

// button debounce time in ms
#define DEBOUNCE_TIME 50

enum ButtonState { BUTTON_RELEASED, BUTTON_PRESSED };

void taskButton();