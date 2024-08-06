/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/8ada83e8a572f8f5f3fa828a6459efc6a0985f04/lib/hack/taskSerial.cpp
 */

#include <FS.h>

#include <printUtilities.h>  // todo: change this, lib should not depend on src....

#include "./globalConfig.h"
#include "./taskWire.h"
#include "./utilities/fileUtilities.h"
#include "./utilities/params.h"
#include "./utilities/serialUtilities.h"
#include "./utilities/wifiUtilities.h"

#define SERIAL_BUFFER_LENGTH 256
#define SERIAL_MAX_PARAM_VALUE_LENGTH 256
char serialBuffer[SERIAL_BUFFER_LENGTH];
uint8_t serialBufferPosition = 0;

void printResult(char* data, Print* output);

void processSpecificCommand(char* data, char* paramValue, Print* output);
void printSpecificHelp(Print* output);

void TaskSerial(void* pvParameters) {
  Serial.begin(115200);
  while (true) {
    debugProcess("TaskSerial");

    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();

      if (inChar == 13 || inChar == 10) {
        // this is a carriage return;
        if (serialBufferPosition > 0) {
          printResult(serialBuffer, &Serial);
        }
        serialBufferPosition = 0;
        serialBuffer[0] = '\0';
      } else {
        if (serialBufferPosition < SERIAL_BUFFER_LENGTH) {
          serialBuffer[serialBufferPosition] = inChar;
          serialBufferPosition++;
          if (serialBufferPosition < SERIAL_BUFFER_LENGTH) {
            serialBuffer[serialBufferPosition] = '\0';
          }
        }
      }
    }
    vTaskDelay(5);
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.

  This method will mainly set/read the parameters:
  Uppercase + number + CR ('-' and 1 to 5 digit) store a parameter
  (0 to 25 depending the letter, starting 26 using to letter like 'AA')
  example: A100, A-1
  -> Many parameters may be set at once
  example: C10,20,30,40,50
  Uppercase + CR read the parameter
  example: A
  -> Many parameters may be read at once
  example: A,B,C,D

  It is also possible to write some data to a specific I2C device using
  nnnRRxxx where
  * nnn : the I2C device number
  * RR : the register to write as a letter: A for 0, B for 1, etc.
  * xxx : a number

  s : read all the parameters
  h : help
  l : show the log file
*/

void printResult(char* data, Print* output) {
  bool theEnd = false;
  uint8_t paramCurrent = 0;  // Which parameter are we defining
  char paramValue[SERIAL_MAX_PARAM_VALUE_LENGTH];
  paramValue[0] = '\0';
  uint8_t paramValuePosition = 0;
  uint8_t i = 0;
  bool inValue = false;
  uint8_t wireTargetAddress = 0;  // used for the command like '55D123'

  while (!theEnd) {
    byte inChar = data[i];
    i++;
    if (i == SERIAL_BUFFER_LENGTH)
      theEnd = true;
    if (inChar == '\0') {
      theEnd = true;
    } else if ((inChar > 47 && inChar < 58) || inChar == '-' ||
               inValue) {  // a number (could be negative)
      if (paramValuePosition < SERIAL_MAX_PARAM_VALUE_LENGTH) {
        paramValue[paramValuePosition] = inChar;
        paramValuePosition++;
        if (paramValuePosition < SERIAL_MAX_PARAM_VALUE_LENGTH) {
          paramValue[paramValuePosition] = '\0';
        }
      }
    } else if (inChar > 64 &&
               inChar < 92) {  // an UPPERCASE character so we define the field
      // we extend however the code to allow 2 letters fields !!!
      if (paramCurrent > 0) {
        paramCurrent *= 26;
      } else {  // do we have a number before the uppercase ????
        if (paramValuePosition > 0) {
          // we have a target I2C device
          wireTargetAddress = atoi(paramValue);
          paramValuePosition = 0;
          paramValue[paramValuePosition] = '\0';
        }
      }
      paramCurrent += inChar - 64;
      if (paramCurrent > MAX_PARAM && wireTargetAddress == 0) {
        paramCurrent = 0;
      }
    }

    if (inChar == ',' || theEnd) {  // store value and increment
      if (paramCurrent > 0) {
        if (paramValuePosition > 0) {
          if (wireTargetAddress > 0) {
#ifdef THR_WIRE_MASTER  // write parameters to I2C follower we designed
            wireWriteIntRegister(wireTargetAddress, paramCurrent - 1,
                                 atoi(paramValue));
#endif
          } else {
            setAndSaveParameter(paramCurrent - 1, atoi(paramValue));
          }
        }
        if (wireTargetAddress > 0) {
#ifdef THR_WIRE_MASTER
          output->println(
              wireReadIntRegister(wireTargetAddress, paramCurrent - 1));
#endif
        } else {
          output->println(parameters[paramCurrent - 1]);
        }
        if (paramCurrent <= MAX_PARAM) {
          paramCurrent++;
          paramValuePosition = 0;
          paramValue[0] = '\0';
        }
      }
    }
    // we may have one or 2 lowercasee
    if (data[0] > 96 && data[0] < 123 &&
        (i > 1 || data[1] < 97 || data[1] > 122)) {
      inValue = true;
    }
  }

  // we will process the commands, it means it starts with lowercase
  switch (data[0]) {
    case 'd':
      printDebug(output);
      break;

    case 'h':
      printHelp(output);
      break;

    case 'f':
      processFSCommand(data[1], paramValue, output);
      break;
    case 'i':
      processWireCommand(data[1], paramValue, output);
      break;
#ifdef THR_EEPROM_LOGGER
    case 'l':
      processLoggerCommand(data[1], paramValue, output);
      break;
#endif
    case 'm':
      processModesCommand(data[1], paramValue, output);
      break;
    case 'p':
      processPrintCommand(data[1], paramValue, output);
      break;
    case 's':
      printParameters(output);
      break;

    case 'u':
      processUtilitiesCommand(data[1], paramValue, output);
      break;
    case 'w':
      processWifiCommand(data[1], paramValue, output);
      break;
      /*
      // default:
      // todo   processSpecificCommand(data, paramValue, output);
      */
  }
  output->println("");
}

void noThread(Print* output) {
  output->println(F("No Thread"));
}

void taskSerial() {
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(TaskSerial, "TaskSerial",
                          4096,  // This stack size can be checked & adjusted by
                                 // reading the Stack Highwater
                          NULL,
                          1,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}
