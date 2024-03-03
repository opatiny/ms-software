#include "Arduino.h"
void TaskBlink(void* pvParameters) {
  (void)pvParameters;

  pinMode(LED_BUILTIN, OUTPUT);

  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(500);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(500);
  }
}

void taskBlink() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskBlink, "TaskBlink",
                          1024,  // Crashes if less than 1024 !!!!
                                 // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          0,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1); // 1 specifies the core
}
