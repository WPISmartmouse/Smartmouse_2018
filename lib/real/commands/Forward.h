#pragma once

#include "CommanDuino.h"
#include "RealMouse.h"
#include "Pose.h"

class Forward : public Command {
  public:
    Forward();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    RealMouse *mouse;
    float forwardDisplacement(Pose p0, Pose p1);

    Pose start;
    float disp;

};
