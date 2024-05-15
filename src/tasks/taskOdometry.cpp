
#include <globalConfig.h>

void updateOdometry(Robot* robot);

void TaskOdometry(void* pvParameters) {
  // left encoder
  while (true) {
    vTaskDelay(1);
    updateOdometry();
  }
}

void taskOdometry() {
  xTaskCreatePinnedToCore(TaskOdometry, "TaskOdometry",
                          4096,  // This stack size can be checked & adjusted
                                 // by reading the Stack Highwater
                          NULL,
                          2,  // Priority, with 3 (configMAX_PRIORITIES - 1)
                              // being the highest, and 0 being the lowest.
                          NULL, 1);
}

void updateOdometry(Robot* robot) {
  // get the current time
  uint32_t now = millis();
  // get the time elapsed since the last update
  float dt = (now - robot->lastUpdate) / 1000.0;
  // update the last update time
  robot->lastUpdate = now;

  // get the left and right encoder values
  int32_t leftEncoder = robot->leftEncoder->read();
  int32_t rightEncoder = robot->rightEncoder->read();

  // calculate the distance traveled by each wheel
  float leftDistance =
      (leftEncoder - robot->lastLeftEncoder) * robot->encoderDistancePerPulse;
  float rightDistance =
      (rightEncoder - robot->lastRightEncoder) * robot->encoderDistancePerPulse;

  // update the last encoder values
  robot->lastLeftEncoder = leftEncoder;
  robot->lastRightEncoder = rightEncoder;

  // calculate the distance traveled by the robot
  float distance = (leftDistance + rightDistance) / 2.0;

  // calculate the change in orientation of the robot
  float dTheta = (rightDistance - leftDistance) / robot->wheelBase;

  // update the orientation of the robot
  robot->theta += dTheta;

  // normalize the orientation to be between -pi and pi
  while (robot->theta > PI) {
    robot->theta -= 2 * PI;
  }
  while (robot->theta < -PI) {
    robot->theta += 2 * PI;
  }

  // calculate the change in x and y position of the robot
  float dx = distance * cos(robot->theta);
  float dy = distance * sin(robot->theta);

  // update the x and y position of the robot
  robot->x += dx;
  robot->y += dy;

  // update the linear and angular velocities of the robot
  robot->v = distance / dt;
  robot->w = dTheta / dt;
}
