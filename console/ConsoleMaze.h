#pragma once
#ifdef CONSOLE

#include <fstream>
#include <string>
#include "SensorReading.h"
#include "AbstractMaze.h"

class ConsoleMaze : public AbstractMaze {
  public:
    ConsoleMaze(std::fstream& fs);
};
#endif
