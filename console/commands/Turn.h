#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/core/Direction.h>

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

