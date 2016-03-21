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
    float yawDiff(float y1, float y2);

    Pose start;
    float disp;

    float l,r;
    bool checkedWalls;
    float *distances;
    bool walls[4];
    bool wallOnLeft, wallOnRight;
    const float kPWall = 2000;
    const float kPDisp= 3000;

};
