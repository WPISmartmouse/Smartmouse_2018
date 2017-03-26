#include "Finish.h"
#include <Arduino.h>

Finish::Finish(AbstractMaze *maze) : Command("end"), maze(maze) {}

void Finish::initialize() {
  Serial.println("end.");
  Serial.println(maze->fastest_route);
}

bool Finish::isFinished() {
  return true;
}
