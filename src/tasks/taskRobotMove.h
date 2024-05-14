#pragma once

enum RobotMode {
  ROBOT_STOP,              // 0
  ROBOT_MOVE,              // 1
  ROBOT_TURN_IN_PLACE,     // 2
  ROBOT_TURN_WITH_RADIUS,  // 3
  ROBOT_MOVE_DISTANCE,     // 4
};

void taskRobotMove();