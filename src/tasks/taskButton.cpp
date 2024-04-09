#include <Arduino.h>

#include <globalConfig.h>
#include <utilities/params.h>

#include "./taskButton.h"

#define LED_PIN D3

void buttonRoutine();

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
    }
  }
}