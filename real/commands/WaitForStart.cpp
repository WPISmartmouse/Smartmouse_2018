#include "WaitForStart.h"
#include "Calibrate.h"
#include "MeasureMazeOrientation.h"

WaitForStart::WaitForStart() : CommandGroup("wait") {
  addSequential(new Calibrate());
  addSequential(new MeasureMazeOrientation());
}
