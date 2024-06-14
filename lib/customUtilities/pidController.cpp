#include "pidController.h"
#include <utilities/params.h>

#include <Arduino.h>

#include <timeUtilities.h>

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
  regulator->previousTime = micros();
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
  uint32_t time = micros();
  double dt = microsToSeconds(time - regulator->previousTime);
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

void clearController(PidController* regulator) {
  regulator->integral = 0;
  regulator->previousError = 0;
  regulator->previousTime = micros();
  regulator->previousValue = 0;
  regulator->correction = 0;
}