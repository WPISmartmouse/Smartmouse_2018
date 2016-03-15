#pragma once

#include "Mouse.h"
#include "SensorReading.h"

class ConsoleMouse : public Mouse {
  public:
    ConsoleMouse(AbstractMaze *maze);
    ConsoleMouse(AbstractMaze *maze, int starting_row, int starting_col);
    virtual SensorReading sense() override;
};
