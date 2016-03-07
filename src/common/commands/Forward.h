#pragma once

#include "CommanDuino.h"
#include "Mouse.h"

class Forward : public Command {
  public:
    Forward(Mouse *mouse);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    Mouse *mouse;

};
