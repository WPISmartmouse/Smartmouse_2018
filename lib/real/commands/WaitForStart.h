#pragma once
#include "CommanDuino.h"
#include "RealMouse.h"

class WaitForStart : public Command {
  public:
    WaitForStart();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    RealMouse *mouse;
};
