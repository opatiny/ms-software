#include "./tasks/globalConfig.h"
#include "./tasks/utilities/params.h"

// functions prototypes
void taskBlink();
void taskSerial();

void setup(){
  taskSerial();
  taskBlink();
}

void loop(){
  vTaskDelay(1000);
}

void resetParameters() {
  for (byte i = 0; i < MAX_PARAM; i++) {
    setAndSaveParameter(i, ERROR_VALUE);
  }
}