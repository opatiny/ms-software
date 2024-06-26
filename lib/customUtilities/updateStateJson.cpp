#include <ArduinoJson.h>

#include "updateStateJson.h"

JsonDocument stateJson;

/**
 * Update the stateJson object with the current state of the robot and place the
 result in tempString.
 *
 * @param robot - The robot structure.
 * @param tempString - The string to store the JSON object.
*/
void getStateString(Robot* robot, char tempString[TEMP_STRING_SIZE]) {
  // IMU data
  stateJson["imu"]["acceleration"]["x"] = robot->imuData.acceleration.x;
  stateJson["imu"]["acceleration"]["y"] = robot->imuData.acceleration.y;
  stateJson["imu"]["acceleration"]["z"] = robot->imuData.acceleration.z;
  stateJson["imu"]["rotation"]["x"] = robot->imuData.rotation.x;
  stateJson["imu"]["rotation"]["y"] = robot->imuData.rotation.y;
  stateJson["imu"]["rotation"]["z"] = robot->imuData.rotation.z;

  // Distance sensors
  for (int i = 0; i < NB_DISTANCE_SENSORS; i++) {
    stateJson["distances"][i] = robot->distances[i];
  }

  // Odometry
  stateJson["odometry"]["pose"]["x"] = robot->odometry.pose.x;
  stateJson["odometry"]["pose"]["y"] = robot->odometry.pose.y;
  stateJson["odometry"]["pose"]["theta"] = robot->odometry.pose.theta;
  stateJson["odometry"]["speed"]["v"] = robot->odometry.speed.v;
  stateJson["odometry"]["speed"]["omega"] = robot->odometry.speed.omega;
  stateJson["odometry"]["time"] = robot->odometry.time;

  serializeJson(stateJson, tempString, TEMP_STRING_SIZE);
}