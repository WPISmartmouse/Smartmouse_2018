#pragma once

#include "Direction.h"
#include <array>

class SensorReading {
  public:
    SensorReading(int row, int col);
    SensorReading(int row, int col, bool *walls);
    std::array<bool, 4> walls;
    const bool isWall(Direction dir);
    const int row, col;
};
