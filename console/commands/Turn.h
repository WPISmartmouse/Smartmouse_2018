#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>
#include <common/Direction.h>

class Turn : public Command {
public:
  Turn(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  Mouse *mouse;
  Direction dir;

};

