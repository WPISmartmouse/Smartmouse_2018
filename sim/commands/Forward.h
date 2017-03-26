#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include "common/commanduino/Command.h"
#include "sim/SimMouse.h"
#include "common/Mouse.h"

class Forward : public Command {
  public:
    Forward();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    double forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1);
    double yawDiff(double y1, double y2);

    ignition::math::Pose3d start;
    double disp;
    SimMouse *mouse;

    double l,r;
    bool checkedWalls;
    SimMouse::RangeData range_data;
    bool walls[4];
    bool wallOnLeft, wallOnRight;
    const float kPWall = 0;
    const float kPDisp = 4;
};
#endif
