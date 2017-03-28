#ifdef SIM

#include "ArcTurn.h"

ArcTurn::ArcTurn(Direction dir) : Command("SimArcTurn"), mouse(SimMouse::inst()), dir(dir) {}

void ArcTurn::initialize() {
  setTimeout((unsigned long) TURN_TIME_MS);
  left = (dir == left_of_dir(mouse->getDir()));
}

void ArcTurn::execute() {
  // calculate speed in radians/sec
  constexpr double inner_rps = (M_PI * TURN_RAD_INNER) / (2 * TURN_TIME_S * SimMouse::WHEEL_RAD);
  constexpr double outer_rps = (M_PI * TURN_RAD_OUTER) / (2 * TURN_TIME_S * SimMouse::WHEEL_RAD);
  double inner_mps = SimMouse::radPerSecToMetersPerSec(inner_rps);
  double outer_mps = SimMouse::radPerSecToMetersPerSec(outer_rps);

  printf("%f, %f\n", inner_mps, outer_mps);

  if (left) {
    mouse->setSpeed(inner_mps, outer_mps);
  }
  else {
    mouse->setSpeed(outer_mps, inner_mps);
  }
}

bool ArcTurn::isFinished() {
  return isTimedOut();
}

void ArcTurn::end() {
  mouse->internalTurnToFace(dir);
}

#endif

