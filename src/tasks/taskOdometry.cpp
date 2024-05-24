#include <globalConfig.h>
#include <utilities/params.h>

#include "../hardwareProperties.h"
#include "../state.h"
#include "../utilities/kinematics.h"

#define SPEED_CALIBRATION_DELAY 1000
#define SPEED_STEP 10
#define MAX_MOTOR_SPEED 255

// delay between each time the debug information is printed
#define DEBUG_DELAY 250

void updateOdometry(Robot* robot);

void TaskOdometry(void* pvParameters) {
  int previousTime = millis();
  robot.odometry.time = previousTime;
  int speed = 0;
  while (true) {
    updateOdometry(&robot);

    if (getParameter(PARAM_DEBUG) == DEBUG_SPEED_CALIBRATION) {
      int currentTime = millis();
      if (currentTime - previousTime > SPEED_CALIBRATION_DELAY) {
        Serial.print(currentTime);
        Serial.print(", ");
        Serial.print(speed);
        Serial.print(", ");
        Serial.print(robot.odometry.leftWheelSpeed);
        Serial.print(", ");
        Serial.println(robot.odometry.rightWheelSpeed);

        speed += SPEED_STEP;
        if (speed > MAX_MOTOR_SPEED) {
          Serial.println("Speed calibration finished.");
          setParameter(PARAM_DEBUG, NO_DEBUG);
          speed = 0;
        }
        setAndSaveParameter(PARAM_MOTOR_LEFT_SPEED_CMD, speed);
        setAndSaveParameter(PARAM_MOTOR_RIGHT_SPEED_CMD, speed);
        previousTime = currentTime;
      }
    }

    if (millis() - previousTime > DEBUG_DELAY &&
        getParameter(PARAM_DEBUG) == DEBUG_ODOMETRY) {
      Serial.print(robot.odometry.pose.x);
      Serial.print(", ");
      Serial.print(robot.odometry.pose.y);
      Serial.print(", ");
      Serial.print(robot.odometry.pose.theta);
      Serial.print(", ");
      Serial.print(robot.odometry.speed.v);
      Serial.print(", ");
      Serial.println(robot.odometry.speed.omega);
      previousTime = millis();
    }
    vTaskDelay(100);
  }
}

void taskOdometry() {
  xTaskCreatePinnedToCore(TaskOdometry, "TaskOdometry",
                          8192,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          3,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);  // attached on core 2!!
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

  Serial.print("leftCounts: ");
  Serial.println(leftCounts);
  Serial.print("dt: ");
  Serial.println(dt);
  Serial.print("leftWheelSpeed: ");
  Serial.println(computeWheelRpm(leftCounts, dt));
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
