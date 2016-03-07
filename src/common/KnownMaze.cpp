#include "KnownMaze.h"
#include "Mouse.h"

KnownMaze::KnownMaze(Mouse *mouse) : AbstractMaze(mouse) {}

bool KnownMaze::is_mouse_blocked(){
  SensorReading sr = sense();
  return sr.isWall(mouse->getDir());
}

bool KnownMaze::is_mouse_blocked(Direction dir){
  SensorReading sr = sense();
  return sr.isWall(dir);
}
