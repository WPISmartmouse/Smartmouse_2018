#include <common/commands/NavTestCommand.h>
#include <commands/WaitForStart.h>
#include <commands/Forward.h>
#include <commands/Stop.h>
#include <commands/Turn.h>

NavTestCommand::NavTestCommand() : CommandGroup("NavTestGroup") {
  addSequential(new WaitForStart());
}
