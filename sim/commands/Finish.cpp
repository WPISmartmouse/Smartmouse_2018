#include <sim/lib/SimMouse.h>
#include "Finish.h"

Finish::Finish(AbstractMaze *maze) : Command("end"), maze(maze) {}

void Finish::initialize() {
  SimMouse::inst()->setSpeedCps(0, 0);
  print("end. Solution = %s\n", maze->fastest_route);
}

bool Finish::isFinished() {
  return true;
}
