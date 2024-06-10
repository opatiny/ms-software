#pragma once

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

typedef struct {
  PidParams params;
  PidSerialParameters serialParams;
  double integral;
  double previousError;
  double previousTime;
  double previousValue;
  double targetValue;
  double correction;
} PidController;

void initialisePidController(PidController* regulator,
                             PidSerialParameters* params);
double getNewPidValue(PidController* regulator, double error);
void updatePidParameters(PidController* regulator);