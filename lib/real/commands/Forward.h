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
    const float kPDisp = 1000;
    const float kPWall = 65;
};
