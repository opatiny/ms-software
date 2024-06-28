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
void printSpeeds(Robot* robot);

void TaskOdometry(void* pvParameters) {
  uint32_t previousTime = millis();
  robot.odometry.time = micros();
  while (true) {
    debugProcess("TaskOdometry ");
    updateOdometry(&robot);

    if (millis() - previousTime > DEBUG_DELAY) {
      switch (getParameter(PARAM_DEBUG)) {
        case DEBUG_ODOMETRY:
          printOdometry(&robot);
          break;
        case DEBUG_SPEEDS:
          printSpeeds(&robot);
          break;
        default:
          break;
      }

      previousTime = millis();
    }
    // problem: what is the optimal delay?
    // if delay is too small, there can be not a single interrupt on the motors
    // pins and the speeds are nonsense if delay is too big, the odometry will
    // have a big error
    vTaskDelay(1);
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

  // calculate high speed of each wheel in rpm
  robot->leftMotor.wheelSpeeds.highSpeed = computeWheelRpm(leftCounts, dt);
  robot->rightMotor.wheelSpeeds.highSpeed = computeWheelRpm(rightCounts, dt);

  // calculate low speed of each wheel in rpm
  robot->leftMotor.wheelSpeeds.lowSpeed =
      getLowSpeedRpm(robot->leftEncoder.lowSpeed);
  robot->rightMotor.wheelSpeeds.lowSpeed =
      getLowSpeedRpm(robot->rightEncoder.lowSpeed);

  // pick what speed to use // todo: change this
  robot->leftMotor.wheelSpeed = robot->leftMotor.wheelSpeeds.lowSpeed;
  robot->rightMotor.wheelSpeed = robot->rightMotor.wheelSpeeds.lowSpeed;

  // calculate the distance traveled by each wheel in m
  double leftDistance = leftCounts * DISTANCE_PER_COUNT;
  double rightDistance = rightCounts * DISTANCE_PER_COUNT;

  // update the last encoder values
  robot->leftEncoder.previousCounts = leftEncoder;
  robot->rightEncoder.previousCounts = rightEncoder;

  // calculate the distance traveled by the robot in m
  double distance = (leftDistance + rightDistance) / 2.0;

  // calculate the change in orientation of the robot in rad (approximation
  // sin(x) = x)
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
  if (dt == 0) {
    dt = 0.000001;
    Serial.println("updateOdometry: dt is 0");
  }

  const double leftRpm = robot->leftMotor.wheelSpeed;
  const double rightRpm = robot->rightMotor.wheelSpeed;
  robot->odometry.speed.v =
      (leftRpm + rightRpm) * WHEEL_DIAMETER * PI / 2.0 / 60.0;  // m/s
  robot->odometry.speed.omega =
      (rightRpm - leftRpm) * WHEEL_DIAMETER / WHEEL_BASE / 60.0;  // rad/s

  double oldV = distance / dt;
  double oldOmega = dTheta / dt;

  if (getParameter(PARAM_DEBUG) == DEBUG_ODOMETRY) {
    Serial.print(oldV);
    Serial.print(", ");
    Serial.print(oldOmega);
    Serial.print(", ");
    Serial.print(robot->odometry.speed.v);
    Serial.print(", ");
    Serial.println(robot->odometry.speed.omega);
  }

  // update the odometry data
  robot->odometry.time = now;
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
}

void printSpeeds(Robot* robot) {
  Serial.print(robot->odometry.time);
  Serial.print(", ");
  Serial.print(robot->leftMotor.wheelSpeeds.highSpeed);
  Serial.print(", ");
  Serial.print(robot->leftMotor.wheelSpeeds.lowSpeed);
  Serial.print(", ");
  Serial.print(robot->rightMotor.wheelSpeeds.highSpeed);
  Serial.print(", ");
  Serial.println(robot->rightMotor.wheelSpeeds.lowSpeed);
}
