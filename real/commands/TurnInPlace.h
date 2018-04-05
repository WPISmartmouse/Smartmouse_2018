#pragma once
#include <common/commanduino/CommanDuino.h>
#include <common/core/Direction.h>

#include <real/RealMouse.h>
#include <common/KinematicController/VelocityProfile.h>

class TurnInPlace : public Command {
public:
  explicit TurnInPlace(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  double goal_yaw;
  double yaw_error;
  bool left_turn;
  RealMouse *mouse;
  Direction dir;


  const static double kP;
  smartmouse::kc::VelocityProfile *profile;
};

