#pragma once
#ifdef CONSOLE

#include <fstream>
#include "SensorReading.h"
#include "KnownMaze.h"

class ConsoleMaze : public KnownMaze {
  public:
    ConsoleMaze(std::fstream& fs, Mouse *mouse);
    ConsoleMaze(Mouse *mouse);
    virtual SensorReading sense() override;
};
#endif
