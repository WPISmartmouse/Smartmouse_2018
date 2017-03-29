#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/AbstractMaze.h>
#include <real/RealMouse.h>

class Finish : public Command {
public:
  Finish(AbstractMaze *maze);

  void initialize();

  bool isFinished();

private:
  AbstractMaze *maze;
  RealMouse *mouse;

};
