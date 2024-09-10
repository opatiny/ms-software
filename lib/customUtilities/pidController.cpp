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
                             PidInitParameters* params) {
  regulator->serialParams.kp = params->serialParams.kp;
  regulator->serialParams.ki = params->serialParams.ki;
  regulator->serialParams.kd = params->serialParams.kd;
  regulator->factor = params->factor;
  regulator->previousTime = micros();
  regulator->modeParameter = params->modeParameter;

  updatePidParameters(regulator);
}

/**
 * Get the new PID value based on the error.
 * @param regulator The PID controller.
 * @param error The error value.
 * @return The new PID value. It is the correction that had to be added to the
 * previous value.
 */
double getNewPidValue(PidController* regulator, double error) {

if(regulator->previousMode != getParameter(regulator->modeParameter) || regulator->clearController == 1) {
    clearController(regulator);
    regulator->previousMode = getParameter(regulator->modeParameter);
    regulator->clearController = 0;
}

  updatePidParameters(regulator);
  uint32_t time = micros();
  double dt = microsToSeconds(time - regulator->previousTime);
  regulator->integral += error * dt;
  double derivative = 0;
  // protect from division by zero
  if (dt != 0) {
    derivative = (error - regulator->previousError) / dt;
  }
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
  regulator->params.kp =
      (double)getParameter(regulator->serialParams.kp) / regulator->factor;
  regulator->params.ki =
      (double)getParameter(regulator->serialParams.ki) / regulator->factor;
  regulator->params.kd =
      (double)getParameter(regulator->serialParams.kd) / regulator->factor;
}

void clearController(PidController* regulator) {
  regulator->integral = 0;
  regulator->previousError = 0;
  regulator->previousTime = micros();
  regulator->previousValue = 0;
  regulator->correction = 0;
}