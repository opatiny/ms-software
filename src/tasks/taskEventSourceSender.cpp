/**
 * Thread allowing to open an HTTP connection and hence send data continuously.
 * This data can then be processed and displayed in the web page.
 *
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/467abdbf719402becc9ab3451b904a775f63663e/src/taskEventSender.cpp
 */

#include <WiFi.h>
#include "./globalConfig.h"
#include "./utilities/params.h"

void sendEventSource(char* event, char* data);

char* tempString = new char[50];

void TaskEventSourceSender(void* pvParameters) {
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(5000);
  }

  while (true) {
    sprintf(tempString, "%i,%i,%i,%i,%i,%i", getParameter(PARAM_ACCELERATION_X),
            getParameter(PARAM_ACCELERATION_Y),
            getParameter(PARAM_ACCELERATION_Z), getParameter(PARAM_ROTATION_X),
            getParameter(PARAM_ROTATION_Y), getParameter(PARAM_ROTATION_Z));
    sendEventSource("accelerometer", tempString);
    vTaskDelay(10);
  }
}

void taskEventSourceSender() {
  vTaskDelay(2000);
  // Now set up two tasks to rntpdun independently.
  xTaskCreatePinnedToCore(TaskEventSourceSender, "TaskEventSourceSender",
                          20000,  // This stack size can be checked & adjusted
                                  // by reading the Stack Highwater
                          NULL,
                          3,  // CRASH if not priority 3 !!!
                          NULL, 1);
}
