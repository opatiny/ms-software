#include <Arduino.h>
#include "../../src/pinMapping.h"

void TaskBlink(void* pvParameters) {
  (void)pvParameters;
  pinMode(BLINK_LED_PIN, OUTPUT);

  while (true) {
    digitalWrite(BLINK_LED_PIN, HIGH);
    vTaskDelay(500);
    digitalWrite(BLINK_LED_PIN, LOW);
    vTaskDelay(500);
  }
}

void taskBlink() {
  xTaskCreatePinnedToCore(TaskBlink, "TaskBlink",
                          2048,  // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          0,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}
