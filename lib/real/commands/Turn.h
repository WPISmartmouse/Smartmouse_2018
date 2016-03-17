#pragma once

#include "CommanDuino.h"
#include "Mouse.h"
#include "Direction.h"

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
