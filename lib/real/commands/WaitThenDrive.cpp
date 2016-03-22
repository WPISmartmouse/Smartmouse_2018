#include "WaitThenDrive.h"
#include "Forward.h"
#include "WaitForStart.h"

WaitThenDrive::WaitThenDrive() : CommandGroup("wait then drive") {
  addSequential(new WaitForStart());
  addSequential(new Forward());
}
