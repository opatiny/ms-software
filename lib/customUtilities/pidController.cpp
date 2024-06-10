#include "pidController.h"

#include <Arduino.h>

/**
 * @brief Initialise the PID controller.
 */
void initialisePidController(PidController* regulator, PidParams* params) {
  regulator->kp = params->kp;
  regulator->ki = params->ki;
  regulator->kd = params->kd;
  regulator->previousTime = millis() / 1000.0;
}

/**
 * Get the new PID value based on the error.
 * @param regulator The PID controller.
 * @param error The error value.
 * @return The new PID value. It is the correction that had to be added to the
 * previous value.
 */
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
  regulator->correction = newValue;
  return newValue;
}