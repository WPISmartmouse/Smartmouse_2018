#pragma once
#ifdef SIM

#include <ignition/math.hh>
#include "CommanDuino.h"
#include "SimMouse.h"
#include "Direction.h"

class Turn : public Command {
  public:
    Turn(Direction dir);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    double yawDiff(double y1, double y2);

    ignition::math::Pose3d start;
    double goalYaw;
    double dYaw;
    SimMouse *mouse;
    Direction dir;

    double l,r;
    const double kP = 0.15;

};
#endif
