#include <SPIFFS.h>
#include "esp_spiffs.h"
#include "globalConfig.h"
#include "params.h"

void printFSHelp(Print* output) {
  output->println(F("(fd) Directory"));
}

static void printFSDir(Print* output) {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    output->printf("%6d ", file.size());
    output->println(file.path());
    file = root.openNextFile();
  }
}

void processFSCommand(char command,
                      char* paramValue,
                      Print* output) {  // char and char* ??
  switch (command) {
    case 'd':
      printFSDir(output);
      break;

    default:
      printFSHelp(output);
      break;
  }
}
