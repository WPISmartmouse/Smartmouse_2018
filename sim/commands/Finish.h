#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/AbstractMaze.h>

class Finish : public Command {
public:
  Finish(AbstractMaze *maze);

  void initialize();

  bool isFinished();

private:
  AbstractMaze *maze;
};

