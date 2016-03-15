#pragma once
#ifdef CONSOLE

#include <fstream>
#include "SensorReading.h"
#include "AbstractMaze.h"

class ConsoleMaze : public AbstractMaze {
  public:
    ConsoleMaze(std::fstream& fs);
};
#endif
