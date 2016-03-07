#pragma once
#ifdef CONSOLE

#include <fstream>
#include "SensorReading.h"
#include "ConsoleMouse.h"
#include "KnownMaze.h"

class ConsoleMaze : public KnownMaze {
  public:
    ConsoleMaze(std::fstream& fs, ConsoleMouse *mouse);
    ConsoleMaze(ConsoleMouse *mouse);
    virtual SensorReading sense() override;
};
#endif
