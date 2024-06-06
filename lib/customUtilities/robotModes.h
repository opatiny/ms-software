#pragma once

enum RobotMode {
  ROBOT_STOP,              // 0
  ROBOT_MOVE,              // 1
  ROBOT_TURN_IN_PLACE,     // 2
  ROBOT_TURN_WITH_RADIUS,  // 3
  ROBOT_MOVE_DISTANCE,     // 4
  ROBOT_STOP_OBSTACLE,     // 5
  /**
   * Control each wheel separately instead of the robot as a whole.
   */
  ROBOT_EACH_WHEEL,  // 6
  /**
   * PID on wheels speeds to move straight.
   */
  ROBOT_MOVE_STRAIGHT  // 7
};

enum MotorMode {
  MOTOR_STOP,            // 0
  MOTOR_CONSTANT_SPEED,  // 1
  MOTOR_SHORT            // 2
};