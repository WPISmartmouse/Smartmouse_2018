#include "SensorReading.h"

SensorReading::SensorReading(int row, int col) :
        walls({false, false, false, false}),
        row(row),
        col(col) {}

SensorReading::SensorReading(int row, int col, bool *walls) :
        walls({walls[0], walls[1], walls[2], walls[3]}),
        row(row),
        col(col) {}

bool SensorReading::isWall(Direction dir) {
  return walls[static_cast<int>(dir)];
}
