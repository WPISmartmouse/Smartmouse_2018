#ifdef SIM

#include "Turn.h"
#include "TurnInPlace.h"
#include "Forward.h"
#include "Stop.h"
#include "ForwardToCenter.h"

Turn::Turn(Direction dir) : CommandGroup("SimTurnGroup"), mouse(SimMouse::inst()), dir(dir) {}

void Turn::initialize() {
  // if we want a logical 180 turn, we do full stop then turn.
//  if (opposite_direction(mouse->getDir()) == dir) {
//    addSequential(new Stop(200)); // slowly stop
//    addSequential(new TurnInPlace(dir));
//    addSequential(new Stop(50)); // slowly stop
//    addSequential(new Forward());
//  } else {
//    addSequential(new ForwardToCenter()); // slowly stop
//    addSequential(new TurnInPlace(dir));
//    addSequential(new Stop(50)); // slowly stop
//    addSequential(new Forward());
//  }
  if (dir != mouse->getDir()) {
    addSequential(new ForwardToCenter()); // slowly stop
    addSequential(new Stop(100)); // slowly stop
    addSequential(new TurnInPlace(dir));
    addSequential(new Forward());
  }
}

#endif
