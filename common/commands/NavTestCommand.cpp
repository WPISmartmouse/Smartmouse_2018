#include "NavTestCommand.h"
#include "WaitForStart.h"
#include "Forward.h"
#include "Stop.h"

NavTestCommand::NavTestCommand() : CommandGroup("NavTestGroup") {
  addSequential(new WaitForStart());
  addSequential(new Forward());
  addSequential(new Stop(100));
}
