#pragma once

#include <state.h>

void printDebug(Print* output);
void printState(Print* output);
void printArray(Print* output, double* array, int size, int nbDigits = 2);
void printRegressions(Print* output,
                      Regressions* regressions,
                      int nbDigits = 2);
void printRegressionsForMatlab(Print* output, Robot* robot, int nbDigits);
void processPrintCommand(char command, char* paramValue, Print* output);
void processModesCommand(char command, char* paramValue, Print* output);