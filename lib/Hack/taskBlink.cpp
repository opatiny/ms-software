#include <Arduino.h>
#include "../../src/pinMapping.h"
void TaskBlink(void* pvParameters) {
  (void)pvParameters;
  Serial.println("set led pin as output");
  pinMode(LED_PIN, OUTPUT);

  while (true) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("High");
    vTaskDelay(500);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Low");
    vTaskDelay(500);
  }
}

void taskBlink() {
  xTaskCreatePinnedToCore(TaskBlink, "TaskBlink",
                          2048,  // Crashes if less than 1024 !!!!
                                 // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          0,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // 1 specifies the core
}
