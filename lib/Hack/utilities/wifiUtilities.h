/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/wifiUtilities.h
 */
#pragma once
#include "../globalConfig.h"

void printWifiHelp(Print* output);

void processWifiCommand(char command, char* paramValue, Print* output);