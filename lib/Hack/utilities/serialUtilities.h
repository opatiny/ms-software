/**
 * This script is widely based on the following script:
 * https://github.com/Hackuarium/esp32-c3/blob/7b421c03214e549868712c29feab7798132af6ba/lib/hack/wifiUtilities.h
 */
#pragma once

#include "../globalConfig.h"
#include "./wifiUtilities.h"

void resetParameters();

static void printFreeMemory(Print* output);

void processUtilitiesCommand(char command, char* paramValue, Print* output);

void printHelp(Print* output);