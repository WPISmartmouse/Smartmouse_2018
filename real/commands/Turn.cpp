#include "Turn.h"
#include "TurnInPlace.h"
#include "Forward.h"
#include "ArcTurn.h"
#include "ForwardToCenter.h"
#include "WallSmash.h"
#include "Backup.h"

Turn::Turn(Direction dir) : CommandGroup("RealTurnGroup"), mouse(RealMouse::inst()), dir(dir) {}

void Turn::initialize() {
  // if we want a logical 180 turn, we do full stop then turn.
  if (opposite_direction(mouse->getDir()) == dir) {
    addSequential(new ForwardToCenter()); // slowly stop
    addSequential(new TurnInPlace(dir));
    addSequential(new Forward());
  } else if (mouse->getDir() != dir) {
    if (smartmouse::kc::WALL_SMASH) {
      addSequential(new WallSmash()) ;
      addSequential(new TurnInPlace(dir));
      addSequential(new Forward());
    }
    else {
      addSequential(new ForwardToCenter());
      addSequential(new TurnInPlace(dir));
      addSequential(new Forward());
    }
  }
}

