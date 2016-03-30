#pragma once

#include "CommanDuino.h"
#include "RealMouse.h"

class Calibration : public Command {
  public:
    Calibration();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    RealMouse *mouse;
    int down;
    int up;
};
