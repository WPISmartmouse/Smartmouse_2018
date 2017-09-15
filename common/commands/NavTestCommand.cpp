#include "ForwardN.h"
#include "NavTestCommand.h"
#include "WaitForStart.h"
#include "Stop.h"

NavTestCommand::NavTestCommand() : CommandGroup("NavTestGroup") {
  addSequential(new WaitForStart());
  addSequential(new ForwardN(5));
  addSequential(new Stop(100));
}
