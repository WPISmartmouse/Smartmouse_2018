#include "NavTestCommand.h"
#include "Forward.h"
#include "Stop.h"
#include "Turn.h"

NavTestCommand::NavTestCommand() : CommandGroup("NavTestGroup") {
  addSequential(new Forward());
  addSequential(new Turn(Direction::S));
  addSequential(new Forward());
  addSequential(new Turn(Direction::N));
  addSequential(new Forward());
  addSequential(new Turn(Direction::S));
  addSequential(new Forward());
  addSequential(new Turn(Direction::N));
  addSequential(new Forward());
  addSequential(new Turn(Direction::S));
  addSequential(new Forward());
  addSequential(new Turn(Direction::N));
  addSequential(new Stop());
}
