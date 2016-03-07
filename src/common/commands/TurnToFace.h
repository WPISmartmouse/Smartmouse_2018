#pragma once

#include "CommanDuino.h"
#include "Mouse.h"
#include "Direction.h"

class TurnToFace : public Command {
  public:
    TurnToFace(Mouse *mouse, Direction d);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    Mouse *mouse;

};
