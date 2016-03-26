#include "Forward.h"

Forward::Forward() : mouse(RealMouse::inst()) {}

void Forward::initialize(){
  start = mouse->getPose();
  disp = 0.0f;
  mouse->setSpeed(100, 0);
}

float Forward::forwardDisplacement(Pose p0, Pose p1){
  switch(mouse->getDir()){
    case Direction::N:
      return p1.y - p0.y;
      break;
    case Direction::E:
      return p1.x - p0.x;
      break;
    case Direction::S:
      return p0.y - p1.y;
      break;
    case Direction::W:
      return p0.x - p1.x;
      break;
  }
}

void Forward::execute(){
  disp = forwardDisplacement(start,mouse->getPose());
  mouse->run();
}

bool Forward::isFinished(){
  return disp > AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);
}

