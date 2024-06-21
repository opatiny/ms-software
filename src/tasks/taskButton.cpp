#include <Arduino.h>

#include <globalConfig.h>
#include <pinMapping.h>
#include <utilities/params.h>

#include "./taskButton.h"
#include "./taskBuzzer.h"
#include "./taskRgbLed.h"
#include "./taskVl53L1X.h"

struct ButtonFlags buttonFlags;

void buttonRoutine();
void setButtonFlags();

void TaskButton(void* pvParameters) {
  pinMode(BUTTON_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonRoutine, CHANGE);

  setParameter(PARAM_BUTTON, BUTTON_RELEASED);

  while (true) {
    if (getParameter(PARAM_BUTTON) == BUTTON_PRESSED) {
      if (getParameter(PARAM_SOUND) == SOUND_ON) {
        setParameter(PARAM_BUZZER_MODE, BUZZER_SINGLE_NOTE);
      }
    } else if (getParameter(PARAM_BUTTON) == BUTTON_RELEASED) {
    }
    vTaskDelay(100);
  }
}

void taskButton() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskButton, "TaskButton",
                          2048,  // Crashes if less than 1024 !!!!
                                 // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // last param specifies the core
}

uint32_t previousTime = millis();

/**
 * Routine to check if the button has been pressed or released, with debouncing.
 */
void buttonRoutine() {
  uint32_t currentTime = millis();
  if (currentTime - previousTime > DEBOUNCE_TIME) {
    previousTime = currentTime;
    if (getParameter(PARAM_BUTTON) == BUTTON_PRESSED) {
      setParameter(PARAM_BUTTON, BUTTON_RELEASED);
      if (getParameter(PARAM_DEBUG) == DEBUG_BUTTON) {
        Serial.println("Button released");
      }
    } else if (getParameter(PARAM_BUTTON) == BUTTON_RELEASED) {
      setParameter(PARAM_BUTTON, BUTTON_PRESSED);
      if (getParameter(PARAM_DEBUG) == DEBUG_BUTTON) {
        Serial.println("Button pressed");
      }
      setButtonFlags();
      // turn RGB led on once
      rgbLedFlags.buttonPressed = 1;
    }
  }
}

/**
 * Set flags that the button has been pressed so the information can be used in
 * other tasks.
 */
void setButtonFlags() {
  buttonFlags.distanceCalibration = BUTTON_PRESSED;
  buttonFlags.robotMode = BUTTON_PRESSED;
}