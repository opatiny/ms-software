/**
 * Odometry task: computes the position and the speed of the robot.
 *
 * Debug: U10
 */

#include <curveFitting.h>

#include <globalConfig.h>
#include <kinematics.h>
#include <motorProperties.h>
#include <printUtilities.h>
#include <state.h>
#include <utilities/params.h>

#include "taskRobotMove.h"

// delay between each time the debug information is printed
#define DEBUG_DELAY 250  // ms

void updateOdometry(Robot* robot);
void printOdometry(Robot* robot);

void TaskOdometry(void* pvParameters) {
  uint32_t previousTime = millis();
  robot.odometry.time = micros();
  while (true) {
    updateOdometry(&robot);

    if (millis() - previousTime > DEBUG_DELAY &&
        getParameter(PARAM_DEBUG) == DEBUG_ODOMETRY) {
      printOdometry(&robot);
      previousTime = millis();
    }
    // problem: what is the optimal delay?
    // if delay is too small, there can be not a single interrupt on the motors
    // pins and the speeds are nonsense if delay is too big, the odometry will
    // have a big error
    vTaskDelay(100);
  }
}

void taskOdometry() {
  xTaskCreatePinnedToCore(TaskOdometry, "TaskOdometry", 4096, NULL,
                          3,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 0);  // attached on core 0!!
}

void updateOdometry(Robot* robot) {
  // get the current time
  uint32_t now = micros();

  // get the time elapsed since the last update in seconds
  double dt = (now - robot->odometry.time) / 1000000.0;

  // update the last update time
  robot->odometry.time = now;

  // get the left and right encoder counts
  int32_t leftEncoder = robot->leftEncoder.counts;
  int32_t rightEncoder = robot->rightEncoder.counts;

  // nb of counts since last update
  double leftCounts = leftEncoder - robot->leftEncoder.previousCounts;
  double rightCounts = rightEncoder - robot->rightEncoder.previousCounts;

  // calculate speed of each wheel in rpm
  robot->leftMotor.wheelSpeed = computeWheelRpm(leftCounts, dt);
  robot->rightMotor.wheelSpeed = computeWheelRpm(rightCounts, dt);

  // calculate the distance traveled by each wheel
  double leftDistance = leftCounts * DISTANCE_PER_COUNT;
  double rightDistance = rightCounts * DISTANCE_PER_COUNT;

  // update the last encoder values
  robot->leftEncoder.previousCounts = leftEncoder;
  robot->rightEncoder.previousCounts = rightEncoder;

  // calculate the distance traveled by the robot
  double distance = (leftDistance + rightDistance) / 2.0;

  // calculate the change in orientation of the robot
  double dTheta = (rightDistance - leftDistance) / WHEEL_BASE;

  // update the orientation of the robot
  robot->odometry.pose.theta += dTheta;

  // normalize the orientation to be between -pi and pi
  while (robot->odometry.pose.theta > PI) {
    robot->odometry.pose.theta -= 2 * PI;
  }
  while (robot->odometry.pose.theta < -PI) {
    robot->odometry.pose.theta += 2 * PI;
  }

  // calculate the change in x and y position of the robot
  double dx = distance * cos(robot->odometry.pose.theta);
  double dy = distance * sin(robot->odometry.pose.theta);

  // update the x and y position of the robot
  robot->odometry.pose.x += dx;
  robot->odometry.pose.y += dy;

  // update the linear and angular velocities of the robot
  robot->odometry.speed.v = distance / dt;
  robot->odometry.speed.omega = dTheta / dt;
  Serial.println("Odometry updated");
}

/**
 * @brief Print the odometry data of the robot (for debug)
 */
void printOdometry(Robot* robot) {
  Serial.print(robot->odometry.pose.x);
  Serial.print(", ");
  Serial.print(robot->odometry.pose.y);
  Serial.print(", ");
  Serial.print(robot->odometry.pose.theta);
  Serial.print(", ");
  Serial.print(robot->odometry.speed.v);
  Serial.print(", ");
  Serial.print(robot->odometry.speed.omega);
  Serial.print(", ");
  Serial.print(robot->leftMotor.wheelSpeed);
  Serial.print(", ");
  Serial.println(robot->rightMotor.wheelSpeed);
}