#pragma once

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>
#include <common/WallFollower.h>

#include "SimMouse.h"

class ArcTurn : public Command {
public:
  ArcTurn(Direction dir);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  static constexpr double TURN_RAD = AbstractMaze::HALF_UNIT_DIST;
  static constexpr double TURN_RAD_INNER = TURN_RAD - Mouse::TRACK_WIDTH / 2;
  static constexpr double TURN_RAD_OUTER = TURN_RAD + Mouse::TRACK_WIDTH / 2;
  static constexpr double TURN_TIME_MS = 1000;
  static constexpr double TURN_TIME_S = TURN_TIME_MS / 1000.0;
  SimMouse *mouse;
  Direction dir;
  bool left; //false means right
};
