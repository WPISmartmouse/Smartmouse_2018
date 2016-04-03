#pragma once

#include "CommanDuino.h"
#include "Mouse.h"
#include "RealMouse.h"
#include "Pose.h"
#include "Direction.h"

class Turn : public Command {
  public:
    Turn(Direction dir);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    float yawDiff(float y1, float y2);

    Pose start;
    float goalYaw;
    float dYaw;
    RealMouse *mouse;
    Direction dir;

    const float kP = 2;
    const float minimalSpeed = 0.1;

    int useIMU;

};
