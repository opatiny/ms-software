#pragma once

enum RobotMode {
  ROBOT_STOP,               // 0
  ROBOT_MOVE_SAME_COMMAND,  // 1
  ROBOT_MOVE,               // 2
  ROBOT_TURN_IN_PLACE,      // 3
  ROBOT_TURN_WITH_RADIUS,   // 4
  ROBOT_MOVE_DISTANCE,      // 5
  ROBOT_STOP_OBSTACLE,      // 6
  /**
   * Control each wheel separately instead of the robot as a whole.
   */
  ROBOT_EACH_WHEEL,  // 7
  /**
   * PID on wheel speeds to move straight.
   */
  ROBOT_WHEEL_SPEED_CONTROL,  // 8
  ROBOT_SPEED_CONTROL         // 9
};

enum MotorMode {
  MOTOR_STOP,            // 0
  MOTOR_CONSTANT_SPEED,  // 1
  MOTOR_SHORT,           // 2
  MOTOR_SPEED_CONTROL    // 3
};