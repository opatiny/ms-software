/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/efd27f2495bc393c5fdfc86ee29539ac684a800b/lib/hack/taskSerial.h
*/

#include <Wire.h>
#include "globalConfig.h"
#include "./utilities/params.h"
#include "./taskWire.h"

void printResult(char* data, Print* output);
void processSpecificCommand(char* data, char* paramValue, Print* output);
void printSpecificHelp(Print* output);
void TaskSerial(void* pvParameters);
void printResult(char* data, Print* output);
