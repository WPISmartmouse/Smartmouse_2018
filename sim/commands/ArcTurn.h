#pragma once
#ifdef SIM

#include <ignition/math.hh>
#include <common/commanduino/CommanDuino.h>
#include <common/Direction.h>

#include "WallFollower.h"
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
  static constexpr double TURN_RAD_INNER = TURN_RAD - SimMouse::TRACK_WIDTH/2;
  static constexpr double TURN_RAD_OUTER = TURN_RAD + SimMouse::TRACK_WIDTH/2;
  static constexpr double TURN_TIME_MS = 1000;
  static constexpr double TURN_TIME_S = TURN_TIME_MS / 1000.0;
  SimMouse *mouse;
  Direction dir;
  bool left; //false means right
};

#endif
