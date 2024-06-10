#include "pidController.h"
#include <utilities/params.h>

#include <Arduino.h>

#define PID_FACTOR 1000.0

void updatePidParameters(PidController* regulator);

/**
 * @brief Initialise the PID controller.
 */
void initialisePidController(PidController* regulator,
                             PidSerialParameters* params) {
  regulator->serialParams.kp = params->kp;
  regulator->serialParams.ki = params->ki;
  regulator->serialParams.kd = params->kd;
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
  updatePidParameters(regulator);
  double time = millis() / 1000.0;
  double dt = time - regulator->previousTime;
  regulator->integral += error * dt;
  double derivative = (error - regulator->previousError) / dt;
  double newValue = regulator->params.kp * error +
                    regulator->params.ki * regulator->integral +
                    regulator->params.kd * derivative;
  regulator->previousError = error;
  regulator->previousTime = time;
  regulator->correction = newValue;
  return newValue;
}

/**
 * @brief Update the PID parameters from the serial parameters.
 */
void updatePidParameters(PidController* regulator) {
  regulator->params.kp = getParameter(regulator->serialParams.kp) / PID_FACTOR;
  regulator->params.ki = getParameter(regulator->serialParams.ki) / PID_FACTOR;
  regulator->params.kd = getParameter(regulator->serialParams.kd) / PID_FACTOR;
}