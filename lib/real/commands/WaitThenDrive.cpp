#include "WaitThenDrive.h"
#include "Forward.h"
#include "WaitForStart.h"
#include "Turn.h"

WaitThenDrive::WaitThenDrive() : CommandGroup("wait then drive") {
  addSequential(new WaitForStart());
  addSequential(new Forward());

  addSequential(new Turn(Direction::S));
  addSequential(new Forward());
  addSequential(new Turn(Direction::W));
  addSequential(new Forward());

  //addSequential(new Forward());
  //addSequential(new Forward());
  //addSequential(new Forward());
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::W));
  //addSequential(new Forward());
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::S));
  //addSequential(new Forward());
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::E));
  //addSequential(new Forward());
  //addSequential(new Turn(Direction::N));
  //addSequential(new Forward());
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
