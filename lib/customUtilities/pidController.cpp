#include "pidController.h"

#include <Arduino.h>

void initialisePidController(PidController* regulator, PidParams* params) {
  regulator->kp = params->kp;
  regulator->ki = params->ki;
  regulator->kd = params->kd;
  regulator->previousTime = millis() / 1000.0;
}

double getNewPidValue(PidController* regulator, double error) {
  double time = millis() / 1000.0;
  double dt = time - regulator->previousTime;
  regulator->integral += error * dt;
  double derivative = (error - regulator->previousError) / dt;
  double newValue = regulator->kp * error +
                    regulator->ki * regulator->integral +
                    regulator->kd * derivative;
  regulator->previousError = error;
  regulator->previousTime = time;
  return newValue;
}