#pragma once

#include "../globalConfig.h"

#include "./wifiUtilities.h"

void resetParameters();

static void printFreeMemory(Print* output);

void processUtilitiesCommand(char command, char* paramValue, Print* output);

void printHelp(Print* output);