#ifdef SIM
#include "SimMouse.h"
#include "Finish.h"
#include <stdio.h>

Finish::Finish(AbstractMaze *maze) : Command("end"), maze(maze){}

void Finish::initialize() {
  printf("end. Solution = %s\n", maze->fastest_route);
}

bool Finish::isFinished() {
  SimMouse::inst()->setSpeed(0,0);
  return true;
}
#endif
