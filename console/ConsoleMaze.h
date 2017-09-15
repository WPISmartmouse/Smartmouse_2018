#pragma once

#include <fstream>
#include <common/core/AbstractMaze.h>

class ConsoleMaze : public AbstractMaze {
public:
  ConsoleMaze(std::fstream &fs);
};
