#pragma once

#ifdef SIM

#include "CommanDuino.h"
#include "SimMouse.h"
#include "Mouse.h"

#include <ignition/math.hh>

class Forward : public Command {
  public:
    Forward(Mouse *mouse);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    float forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1);

    ignition::math::Pose3d start;
    float disp;
    SimMouse *mouse;

    float l,r;
    const float kP = 3000;

};
#endif
