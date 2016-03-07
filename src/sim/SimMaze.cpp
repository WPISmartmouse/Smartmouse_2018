#ifdef SIM
#include "SimMaze.h"

SimMaze::SimMaze(SimMouse *mouse) : KnownMaze(mouse) {}

void SimMaze::senseCallback(ConstLaserScanStampedPtr &msg){
  //transform from Mouse frame to Cardinal frame;
  int size = msg->scan().ranges_size();
  switch(mouse->getDir()) {
    case Direction::N:
      walls[0] = msg->scan().ranges(1) < WALL_DIST;
      walls[1] = msg->scan().ranges(0) < WALL_DIST;
      walls[2] = msg->scan().ranges(3) < WALL_DIST;
      walls[3] = msg->scan().ranges(2) < WALL_DIST;
      break;
    case Direction::E:
      walls[0] = msg->scan().ranges(2) < WALL_DIST;
      walls[1] = msg->scan().ranges(1) < WALL_DIST;
      walls[2] = msg->scan().ranges(0) < WALL_DIST;
      walls[3] = msg->scan().ranges(3) < WALL_DIST;
      break;
    case Direction::S:
      walls[0] = msg->scan().ranges(3) < WALL_DIST;
      walls[1] = msg->scan().ranges(2) < WALL_DIST;
      walls[2] = msg->scan().ranges(1) < WALL_DIST;
      walls[3] = msg->scan().ranges(0) < WALL_DIST;
      break;
    case Direction::W:
      walls[0] = msg->scan().ranges(0) < WALL_DIST;
      walls[1] = msg->scan().ranges(3) < WALL_DIST;
      walls[2] = msg->scan().ranges(2) < WALL_DIST;
      walls[3] = msg->scan().ranges(1) < WALL_DIST;
      break;
  }
  senseCond.notify_all();
}

SensorReading SimMaze::sense(){
  std::unique_lock<std::mutex> lk(senseMutex);
  senseCond.wait(lk);
  SensorReading sr(mouse->getRow(), mouse->getCol());
  bool *w = sr.walls;
  for (int i=0;i<4;i++){
    *(w++) = walls[i];
  }

  return sr;
}
#endif
