#include "NavTestCommand.h"
#include "Forward.h"
#include "Stop.h"
#include "Turn.h"

NavTestCommand::NavTestCommand() : CommandGroup("NavTestGroup") {
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());
  addSequential(new Forward());

  addSequential(new Stop(150));
}
