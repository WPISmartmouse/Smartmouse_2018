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
    float yawDiff(float y1, float y2);
    void end();

  private:
    bool outOfRange(float range);
    float forwardDisplacement(Pose p0, Pose p1);

    RealMouse *mouse;
    Pose start;
    float disp;
    float *distances;
    bool checkedWalls;
    bool wallOnLeft, wallOnRight;
    bool walls[4];
    float goalYaw;
    const float kPDisp = 2000;
    const float minimalSpeed = 0.005;
    const float kPWall = 65;
    const float kYaw = 0.5;
    const float ignore_wall_region_L = 0.03;
    const float ignore_wall_region_H = 0.06;
};
