#include "Finish.h"

Finish::Finish(AbstractMaze *maze) : Command("end"), maze(maze) {}

void Finish::initialize() {
  std::string s = route_to_string(maze->fastest_route);
  printf("end. Solution=%s\n", s.c_str());
}

bool Finish::isFinished() {
  return true;
}

