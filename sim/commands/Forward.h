#pragma once

#ifdef SIM

#include "CommanDuino.h"
#include "SimMouse.h"
#include "Mouse.h"

#include <ignition/math.hh>

class Forward : public Command {
  public:
    Forward();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    float forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1);
    float yawDiff(float y1, float y2);

    ignition::math::Pose3d start;
    float disp;
    SimMouse *mouse;

    float l,r;
    bool checkedWalls;
    float *distances;
    bool walls[4];
    bool wallOnLeft, wallOnRight;
    const float kPWall = 2000;
    const float kPDisp= 3000;

};
#endif
