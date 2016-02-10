#include "SensorReading.h"

SensorReading::SensorReading(int row, int col) :
  row(row), col(col), walls{false,false,false,false} {}

SensorReading::SensorReading(int row, int col, bool *walls) :
  row(row),
  col(col),
  walls{walls[0], walls[1], walls[2], walls[3]} {
}

const bool SensorReading::isWall(Direction dir){
  return walls[static_cast<int>(dir)];
}
