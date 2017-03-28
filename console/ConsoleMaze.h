#pragma once
#ifdef CONSOLE

#include <fstream>
#include <string>
#include <common/SensorReading.h>
#include <common/AbstractMaze.h>

class ConsoleMaze : public AbstractMaze {
public:
  ConsoleMaze(std::fstream &fs);
};

#endif
