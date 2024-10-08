/**
 * Thread allowing to open an HTTP connection and hence send data continuously.
 * This data can then be processed and displayed in the web page.
 *
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/467abdbf719402becc9ab3451b904a775f63663e/src/taskEventSender.cpp
 */

#include <WiFi.h>

#include <state.h>
#include <updateStateJson.h>
#include "./globalConfig.h"
#include "./taskWebserver.h"
#include "./utilities/params.h"

char tempString[TEMP_STRING_SIZE];

void TaskEventSourceSender(void* pvParameters) {
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(5000);
  }

  while (true) {
    getStateString(&robot, tempString);
    // Serial.println(tempString);
    sendEventSource("state", tempString);
    vTaskDelay(200);
  }
}

void taskEventSourceSender() {
  vTaskDelay(2000);
  // Now set up two tasks to rntpdun independently.
  xTaskCreatePinnedToCore(TaskEventSourceSender, "TaskEvent",
                          20000,  // This stack size can be checked & adjusted
                                  // by reading the Stack Highwater
                          NULL,
                          3,  // CRASH if not priority 3 !!!
                          NULL, 1);
}
