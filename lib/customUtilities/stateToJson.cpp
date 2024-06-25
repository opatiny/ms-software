#include <ArduinoJson.h>

#include <state.h>

JsonDocument state;

void stateToJson(Robot* robot) {
  // IMU data
  state["imu"]["acceleration"]["x"] = robot->imuData.acceleration.x;
  state["imu"]["acceleration"]["y"] = robot->imuData.acceleration.y;
  state["imu"]["acceleration"]["z"] = robot->imuData.acceleration.z;
  state["imu"]["rotation"]["x"] = robot->imuData.rotation.x;
  state["imu"]["rotation"]["y"] = robot->imuData.rotation.y;
  state["imu"]["rotation"]["z"] = robot->imuData.rotation.z;

  // Distance sensors
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    state["distanceSensors"][i] = robot->distances[i];
  }

  // Odometry
  state["odometry"]["pose"]["x"] = robot->odometry.pose.x;
  state["odometry"]["pose"]["y"] = robot->odometry.pose.y;
  state["odometry"]["pose"]["theta"] = robot->odometry.pose.theta;
  state["odometry"]["speed"]["v"] = robot->odometry.speed.v;
  state["odometry"]["speed"]["omega"] = robot->odometry.speed.omega;
  state["odometry"]["time"] = robot->odometry.time;
}