#pragma once

#include <stdint.h>

struct PidParams {
  double kp;
  double ki;
  double kd;
};

struct PidSerialParameters {
  int kp;
  int ki;
  int kd;
};

struct PidInitParameters {
  PidSerialParameters serialParams;
  int factor;
  int modeParameter;
};

typedef struct {
  int factor;
  PidParams params;
  PidSerialParameters serialParams;
  double integral;
  double previousError;
  uint32_t previousTime;  // in us!!
  double previousValue;
  double targetValue;
  double correction;
  int modeParameter;
  int previousMode;
  bool clearController;
} PidController;

void initialisePidController(PidController* regulator,
                             PidInitParameters* params);
double getNewPidValue(PidController* regulator, double error);
void updatePidParameters(PidController* regulator);
void clearController(PidController* regulator);