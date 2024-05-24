#include "pidRegulation.h"

#include <Arduino.h>

double getNewPidValue(PidRegulator* regulator, double error) {
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