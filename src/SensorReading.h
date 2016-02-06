#pragma once

#include "Direction.h"

class SensorReading {
  public:
    SensorReading(int row, int col, bool *walls);
    bool const walls[4];
    const bool isWall(Direction dir);
    const int row, col;
};
