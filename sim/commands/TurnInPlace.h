#pragma once
#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/core/Direction.h>
#include <sim/lib/SimMouse.h>
#include <common/KinematicController/VelocityProfile.h>

class TurnInPlace : public Command {
public:
  TurnInPlace(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  SimMouse *mouse;
  bool left_turn;
  double goal_yaw;
  double yaw_error;
  Direction dir;

  const static double kP;
  smartmouse::kc::VelocityProfile *profile;
};

