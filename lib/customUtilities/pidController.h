#pragma once

typedef struct {
  double kp;
  double ki;
  double kd;
  double integral;
  double previousError;
  double previousTime;
} PidController;

struct PidParams {
  double kp;
  double ki;
  double kd;
};

void initialisePidController(PidController* regulator, PidParams* params);
double getNewPidValue(PidController* regulator, double error);