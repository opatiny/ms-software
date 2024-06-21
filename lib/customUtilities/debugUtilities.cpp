#include <utilities/params.h>

#include "debugUtilities.h"

void debugTask(char const* taskName) {
  if (DEBUG_TASKS_UP) {
    Serial.print("Task ");
    Serial.print(taskName);
    Serial.print(" running, remaining stack: ");
    TaskHandle_t handle = xTaskGetHandle(taskName);
    Serial.println(uxTaskGetStackHighWaterMark(handle));
  }
}

/**
 * @brief Print the process name every time the while loop is executed.
 */
void debugProcess(char const* processName) {
  if (getParameter(PARAM_DEBUG) == DEBUG_PROCESSES) {
    Serial.println(processName);
  }
}