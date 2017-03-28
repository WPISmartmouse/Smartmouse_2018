#include "WaitForStart.h"
#include "Calibrate.h"
#include "MeasureMazeOrientation.h"
#include "RealMouse.h"

WaitForStart::WaitForStart() : CommandGroup("wait") {
  addSequential(new Calibrate());
  addSequential(new MeasureMazeOrientation());
}
