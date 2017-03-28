#ifdef SIM

#include "Turn.h"
#include "TurnInPlace.h"
#include "Forward.h"
#include "Stop.h"
#include "ArcTurn.h"

Turn::Turn(Direction dir) : CommandGroup("SimTurnGroup"), mouse(SimMouse::inst()), dir(dir) {}

void Turn::initialize() {
  // if we want a logical 180 turn, we do full stop then turn.
  if (opposite_direction(mouse->getDir()) == dir) {
    addSequential(new Stop(200)); // slowly stop
    addSequential(new TurnInPlace(dir));
    addSequential(new Forward());
  } else {
    addSequential(new ArcTurn(dir));
//    addSequential(new Forward());
  }
}

#endif
