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
  double goal_yaw;
  double yaw_error;
  SimMouse *mouse;
  Direction dir;

  const static double kP;
  smartmouse::kc::VelocityProfile *left_profile;
  smartmouse::kc::VelocityProfile *right_profile;
};

