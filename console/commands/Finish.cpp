#ifdef CONSOLE

#include "Finish.h"

Finish::Finish(AbstractMaze *maze) : Command("end"), maze(maze) {}

void Finish::initialize() {
  printf("end. Solution=%s\n", maze->fastest_route);
}

bool Finish::isFinished() {
  return true;
}

#endif
