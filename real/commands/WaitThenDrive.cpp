#include <common/commands/Delay.h>

#include "WaitThenDrive.h"
#include "Forward.h"
#include "Turn.h"
#include "WaitForStart.h"

WaitThenDrive::WaitThenDrive() : CommandGroup("wait then drive") {
  addSequential(new WaitForStart());

  addSequential(new Forward());
  addSequential(new Turn(Direction::W));
  addSequential(new Delay(500));
  addSequential(new Forward());
  //addSequential(new Turn(Direction::S));
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::W));
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::N));
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::W));
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::S));
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::W));
  //addSequential(new Forward());
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::N));
  //addSequential(new Forward());
}
