#include <ArduinoJson.h>

#include <utilities/params.h>
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
  stateJson["navigation"]["mode"] =
      getParameter(robot->navigation.modeParameter);
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
  stateJson["odometry"]["time"] = robot->odometry.time;

  // Controllers
  //   Serial.print("v");
  //   Serial.println(>robot->navigation.robotSpeedController.linear.targetValue);
  stateJson["controllers"]["v"]["target"] =
      robot->navigation.robotSpeedController.linear.targetValue;
  stateJson["controllers"]["v"]["current"] = robot->odometry.speed.v;
  stateJson["controllers"]["v"]["kp"] =
      robot->navigation.robotSpeedController.linear.params.kp;
  stateJson["controllers"]["v"]["ki"] =
      robot->navigation.robotSpeedController.linear.params.ki;
  stateJson["controllers"]["v"]["kd"] =
      robot->navigation.robotSpeedController.linear.params.kd;
  stateJson["controllers"]["v"]["mode"] = getParameter(
      robot->navigation.robotSpeedController.modeParameters.linearController);

  stateJson["controllers"]["omega"]["target"] =
      robot->navigation.robotSpeedController.angular.targetValue;
  stateJson["controllers"]["omega"]["current"] = robot->odometry.speed.omega;
  stateJson["controllers"]["omega"]["kp"] =
      robot->navigation.robotSpeedController.angular.params.kp;
  stateJson["controllers"]["omega"]["ki"] =
      robot->navigation.robotSpeedController.angular.params.ki;
  stateJson["controllers"]["omega"]["kd"] =
      robot->navigation.robotSpeedController.angular.params.kd;
  stateJson["controllers"]["omega"]["mode"] = getParameter(
      robot->navigation.robotSpeedController.modeParameters.angularController);

  stateJson["leftMotor"]["command"] = robot->leftMotor.currentCommand;
  stateJson["leftMotor"]["speed"] = robot->leftMotor.wheelSpeed;
  stateJson["rightMotor"]["command"] = robot->rightMotor.currentCommand;
  stateJson["rightMotor"]["speed"] = robot->rightMotor.wheelSpeed;

  serializeJson(stateJson, tempString, TEMP_STRING_SIZE);
}