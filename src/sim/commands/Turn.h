#pragma once
#ifdef SIM

#include "CommanDuino.h"
#include "SimMouse.h"
#include "Direction.h"

#include <ignition/math.hh>

class Turn : public Command {
  public:
    Turn(Mouse *mouse, Direction dir);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    float yawDiff(float y1, float y2);

    ignition::math::Pose3d start;
    float goalYaw;
    float dYaw;
    SimMouse *mouse;
    Direction dir;

    float l,r;
    const float kP = 8;

};
#endif
