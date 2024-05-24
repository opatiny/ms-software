#pragma once

typedef struct {
  double kp;
  double ki;
  double kd;
  double integral;
  double previousError;
  double previousTime;
} PidRegulator;

double getNewPidValue(PidRegulator* regulator, double error);