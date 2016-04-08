#pragma once

#include "CommanDuino.h"
#include "RealMouse.h"
#include "Pose.h"

class Forward : public CommandGroup {
  public:
    Forward();
    void initialize();
    void execute();
    bool isFinished();
    float yawDiff(float y1, float y2);
    void end();

  private:
    enum class FwdState { GO_UNTIL_CHECK, CHECK, STOP_AT_WALL, STOP_AT_DIST};
    FwdState state;
    bool outOfRange(float range);
    float forwardDisplacement(Pose p0, Pose p1);

    RealMouse *mouse;
    Pose start;
    float disp;
    float *distances;
    bool checkedWalls;
    bool wallOnLeft, wallOnRight;
    float dispError;
    bool walls[4];
    float goalYaw;

    // \brief This is the distance from the front distance
    // sensor to the wall if the mouse is in the center of the square
    const float distFromSensorToWallFromCenter = 0.044;
    const float minFrontDist = 0.08;
    const float maxFrontDist = 0.18;
    const float kPDisp = 3000;
    const float kPWall = 115;
    const float kYaw = 0.4;
};
