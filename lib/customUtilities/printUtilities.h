#pragma once

#include <state.h>

void printDebug();
void printState();
void printArray(double* array, int size, int nbDigits = 2);
void printRegressions(Regressions* regressions, int nbDigits = 2);
void printRegressionsForMatlab(Robot* robot, int nbDigits);
void processPrintCommand(char command, char* paramValue, Print* output);