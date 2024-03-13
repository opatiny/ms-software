#pragma once

#include "globalConfig.h"

boolean setParameterBit(byte number, byte bitToSet);
boolean clearParameterBit(byte number, byte bitToClear);

boolean getParameterBit(byte number, byte bitToRead);

boolean setParameterBit(byte number, byte bitToSet);

boolean clearParameterBit(byte number, byte bitToClear);

void setupParameters();

void toggleParameterBit(byte number, byte bitToToggle);

int16_t getParameter(byte number);

void setParameter(byte number, int16_t value);

void incrementParameter(byte number);

void saveParameters();

void setAndSaveParameter(byte number, int16_t value);

boolean saveAndLogError(boolean isError, byte errorFlag);

void printParameter(Print* output, byte number);

void printParameters(Print* output);

void printCompactParameters(Print* output, byte number);

void printCompactParameters(Print* output);

void getFunction(char* string);
void setFunction(const char* string);

void setParameter(char* key, char* value);
void getParameter(char* key, char* value);

void setQualifier(int16_t value);

#define ERROR_VALUE -32768