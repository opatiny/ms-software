#include "tasks/taskDcMotor.h"
#include "utilities/params.h"

#define GEAR_RATIO 30
#define COUNTS_PER_REV 12
#define WHEEL_DIAMETER 32  // in mm
#define WHEEL_BASE 100     // in mm, distance between centers of wheels

enum Motor { LEFT, RIGHT };

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
void rotateMotor(Motor motor, int degrees, int speed) {
  int counts = angleToCounts(degrees);
  int startCounts = getParameter(PARAM_ENCODER_LEFT);
  int targetCounts = startCounts + counts;
  setParameter(PARAM_MOTOR_LEFT_SPEED, speed);
  setParameter(PARAM_MOTOR_LEFT_MODE,
               speed > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
  if (getParameter(PARAM_ENCODER_LEFT) >= targetCounts) {
    setParameter(PARAM_MOTOR_LEFT_MODE, MOTOR_STOP);
  }
}
