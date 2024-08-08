#pragma once

// print message after each task is initialized
#define DEBUG_TASKS_UP 1

// debug (U)
enum DebugMode {
  NO_DEBUG,                  // 0
  DEBUG_DISTANCE,            // 1
  DEBUG_IMU,                 // 2
  DEBUG_VOLTAGES,            // 3
  DEBUG_BATTERY_LOG_DATA,    // 4
  DEBUG_ENCODERS,            // 5
  DEBUG_BUTTON,              // 6
  DEBUG_MOTORS,              // 7
  DEBUG_RGB_LED,             // 8
  DEBUG_BUZZER,              // 9
  DEBUG_ODOMETRY,            // 10
  DEBUG_SPEED_CALIBRATION,   // 11
  DEBUG_ROBOT_CONTROL,       // 12
  DEBUG_MOTORS_DT,           // 13
  DEBUG_PROCESSES,           // 14
  DEBUG_SPEEDS,              // 15
  DEBUG_ROBOT_ACCELERATION,  // 16
};

void debugTask(char const* taskName);
void debugProcess(char const* processName);