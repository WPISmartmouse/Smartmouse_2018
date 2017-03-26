#pragma once

#include <common/commanduino/Command.h>
#include <common/Direction.h>
#include <real/RealMouse.h>
#include "Pose.h"

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

    constexpr static float MIN_ROT_SPEED = M_PI/12; //rad/s
    const float kP = 0.75;
};
