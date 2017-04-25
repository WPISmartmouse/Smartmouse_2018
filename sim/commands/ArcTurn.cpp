#include "ArcTurn.h"

const double ArcTurn::kTurn = 9.5;

ArcTurn::ArcTurn(Direction dir) : Command("SimArcTurn"), mouse(SimMouse::inst()), goal_dir(dir) {}

void ArcTurn::initialize() {
}

void ArcTurn::execute() {
}

bool ArcTurn::isFinished() {
  return true;
}

void ArcTurn::end() {
  mouse->internalTurnToFace(goal_dir);
}
