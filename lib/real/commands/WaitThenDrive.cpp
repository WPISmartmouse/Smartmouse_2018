#include "WaitThenDrive.h"
#include "Forward.h"
#include "WaitForStart.h"
#include "Turn.h"
#include "Calibrate.h"
#include "MeasureMazeOrientation.h"

WaitThenDrive::WaitThenDrive() : CommandGroup("wait then drive") {
  addSequential(new Calibrate());
  addSequential(new MeasureMazeOrientation());
  addSequential(new WaitForStart());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Turn(Direction::W));
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Turn(Direction::E));
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Turn(Direction::W));
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Turn(Direction::E));
}
