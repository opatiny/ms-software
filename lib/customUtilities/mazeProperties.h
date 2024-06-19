#pragma once

#include <stdbool.h>

#define CELL_SIZE 180
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

// North, East, South, West
typedef bool Walls[4];

struct Cell {
  Walls walls;
  bool visited;
};

struct MazePosition {
  int x;
  int y;
};
