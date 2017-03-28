#pragma once

#include "CommanDuino.h"
#include "RealMouse.h"

class Calibrate : public Command {
  public:
    Calibrate();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    const unsigned long REFRESH_TIME = 200;
    RealMouse *mouse;
    uint32_t lastDisplayUpdate;
};
