#include "./motorCommands.h"
#include <utilities/params.h>

Motor leftMotor = {
  speedParameter : PARAM_MOTOR_LEFT_SPEED,
  modeParameter : PARAM_MOTOR_LEFT_MODE,
  encoderParameter : PARAM_ENCODER_LEFT,
  speed : 0,
  pin1 : MOTOR_LEFT_PIN1,
  pin2 : MOTOR_LEFT_PIN2
};

Motor rightMotor = {
  speedParameter : PARAM_MOTOR_RIGHT_SPEED,
  modeParameter : PARAM_MOTOR_RIGHT_MODE,
  encoderParameter : PARAM_ENCODER_RIGHT,
  speed : 0,
  pin1 : MOTOR_RIGHT_PIN1,
  pin2 : MOTOR_RIGHT_PIN2
};

/**
 * @brief Convert nb of encoder counts to an angle in degrees.
 * todo: Is the return type int a problem?
 */
int countsToAngle(int counts) {
  return counts * 360 / (12 * GEAR_RATIO);
}

/**
 * @brief Convert an angle in degrees to nb of encoder counts.
 */
int angleToCounts(int angle) {
  return angle * 12 * GEAR_RATIO / 360;
}

/**
 * @brief Rotate the specified motor of a given number of degrees.
 * @param motor Motor to rotate.
 * @param degrees Number of degrees to rotate.
 * @param speed Speed of the motor (0 to 255)
 */
void moveDegrees(Motor motor, int degrees, int speed) {
  int counts = angleToCounts(degrees);
  int startCounts = getParameter(motor.encoderParameter);
  int targetCounts = startCounts + counts;
}
