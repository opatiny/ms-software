#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>
#include "../pinMapping.h"

#include "./taskButton.h"
#include "./taskVl53L1X.h"

void buttonRoutine();
void setButtonFlags();

void TaskButton(void* pvParameters) {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonRoutine, CHANGE);

  setParameter(PARAM_BUTTON, BUTTON_RELEASED);

  digitalWrite(LED_PIN, LOW);

  while (true) {
    if (getParameter(PARAM_BUTTON) == BUTTON_PRESSED) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
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
                          NULL, 1);  // 1 specifies the core
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
      if (getParameter(PARAM_DEBUG) == DEBUG_BUTTON) {
        Serial.println("Button released");
      }
      setParameter(PARAM_BUTTON, BUTTON_RELEASED);
    } else if (getParameter(PARAM_BUTTON) == BUTTON_RELEASED) {
      if (getParameter(PARAM_DEBUG) == DEBUG_BUTTON) {
        Serial.println("Button pressed");
      }
      setParameter(PARAM_BUTTON, BUTTON_PRESSED);
      setButtonFlags();
    }
  }
}

/**
 * Set flags that the button has been pressed so the information can be used in
 * other tasks.
 */
void setButtonFlags() {
  distance_calibration_button_pressed = true;
}