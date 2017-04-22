#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/AbstractMaze.h>
#include <real/RealMouse.h>

class Finish : public Command {
public:
  Finish(AbstractMaze *maze);

  void initialize();
  void execute();
  bool isFinished();

private:
  const unsigned int BLINK_TIME = 50;

  AbstractMaze *maze;
  RealMouse *mouse;
  unsigned long t;
  uint8_t pin_id;
  bool on;
};
