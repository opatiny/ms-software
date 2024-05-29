#include <globalConfig.h>
#include <kinematics.h>
#include <utilities/params.h>

#include "../hardwareProperties.h"
#include "../state.h"
#include "taskRobotMove.h"
#include "utilities/speedCalibration.h"

// delay between each time the debug information is printed
#define DEBUG_DELAY 250

void updateOdometry(Robot* robot);
void printOdometry(Robot* robot);
void wheelSpeedCalibration(int* speed, int* previousTime);

void TaskOdometry(void* pvParameters) {
  int previousTime = millis();
  robot.odometry.time = previousTime;
  int speed = MIN_MOTOR_COMMAND;
  while (true) {
    updateOdometry(&robot);

    if (getParameter(PARAM_DEBUG) == DEBUG_SPEED_CALIBRATION) {
      wheelSpeedCalibration(&speed, &previousTime);
    }

    if (millis() - previousTime > DEBUG_DELAY &&
        getParameter(PARAM_DEBUG) == DEBUG_ODOMETRY) {
      printOdometry(&robot);
      previousTime = millis();
    }
    // handle case where user changes debug mode before calibration is finished
    if (getParameter(PARAM_DEBUG) != DEBUG_SPEED_CALIBRATION &&
        speed != MIN_MOTOR_COMMAND) {
      speed = MIN_MOTOR_COMMAND;
      setParameter(PARAM_ROBOT_SPEED_CMD, 0);
      setParameter(PARAM_ROBOT_MODE, ROBOT_STOP);
    }
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
  uint32_t now = millis();
  // get the time elapsed since the last update
  float dt = (now - robot->odometry.time) / 1000.0;

  // update the last update time
  robot->odometry.time = now;

  // get the left and right encoder counts
  int32_t leftEncoder = robot->leftEncoder.counts;
  int32_t rightEncoder = robot->rightEncoder.counts;

  // nb of counts since last update
  float leftCounts = leftEncoder - robot->leftEncoder.previousCounts;
  float rightCounts = rightEncoder - robot->rightEncoder.previousCounts;

  // calculate speed of each wheel in rpm
  robot->odometry.leftWheelSpeed = computeWheelRpm(leftCounts, dt);
  robot->odometry.rightWheelSpeed = computeWheelRpm(rightCounts, dt);

  // calculate the distance traveled by each wheel
  float leftDistance = leftCounts * DISTANCE_PER_COUNT;
  float rightDistance = rightCounts * DISTANCE_PER_COUNT;

  // update the last encoder values
  robot->leftEncoder.previousCounts = leftEncoder;
  robot->rightEncoder.previousCounts = rightEncoder;

  // calculate the distance traveled by the robot
  float distance = (leftDistance + rightDistance) / 2.0;

  // calculate the change in orientation of the robot
  float dTheta = (rightDistance - leftDistance) / WHEEL_BASE;

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
  float dx = distance * cos(robot->odometry.pose.theta);
  float dy = distance * sin(robot->odometry.pose.theta);

  // update the x and y position of the robot
  robot->odometry.pose.x += dx;
  robot->odometry.pose.y += dy;

  // update the linear and angular velocities of the robot
  robot->odometry.speed.v = distance / dt;
  robot->odometry.speed.omega = dTheta / dt;
}

void printOdometry(Robot* robot) {
  Serial.print(robot->odometry.pose.x);
  Serial.print(", ");
  Serial.print(robot->odometry.pose.y);
  Serial.print(", ");
  Serial.print(robot->odometry.pose.theta);
  Serial.print(", ");
  Serial.print(robot->odometry.speed.v);
  Serial.print(", ");
  Serial.println(robot->odometry.speed.omega);
}
