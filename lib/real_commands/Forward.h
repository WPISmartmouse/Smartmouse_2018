#pragma once

#include "CommanDuino.h"
#include "Pose.h"
#include "../real/RealMouse.h"
#include "../KinematicMotorController/Pose.h"

class Forward : public CommandGroup {
  public:
    Forward();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:

    float yawDiff();
    bool outOfRange(float range);
    float forwardDisplacement(Pose p0, Pose p1);
    float calculateRemainingDistance(float dToWallOnLeft, float dToWallRight, float rawFrontWallDist);

    enum class FwdState { GO_UNTIL_CHECK, CHECK, STOP_AT_WALL, STOP_AT_DIST};
    RealMouse *mouse;
    bool checkedWalls;
    FwdState state;
    Pose start;
    float distanceSoFar;
    float *distances;
    float remainingDistance;
    bool walls[4];
    float goalYaw;

    // \brief This is the distance from the front distance
    // sensor to the wall if the mouse is in the center of the square
    const float distFromSensorToWallFromCenter = 0.044;
    constexpr static float MIN_SPEED = 20; //mm/s
    const float WALL_OUT_OF_RANGE_DIST = 0.10; //meters
    const float CHECK_DIST = 0.12; //meters
    const float maxFrontDist = 0.18; //meters
    const float kPDisp = 3000;
    const float kPWall = 115;
};
