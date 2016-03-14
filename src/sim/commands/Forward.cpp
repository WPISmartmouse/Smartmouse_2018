#ifdef SIM

#include "Forward.h"
#include "Direction.h"
#include "AbstractMaze.h"

Forward::Forward(Mouse *mouse) : mouse((SimMouse *)mouse), l(0), r(0) {}

float Forward::forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1){
  switch(mouse->getDir()){
    case Direction::N:
      return p1.Pos().Y() - p0.Pos().Y();
      break;
    case Direction::E:
      return p1.Pos().X() - p0.Pos().X();
      break;
    case Direction::S:
      return p0.Pos().Y() - p1.Pos().Y();
      break;
    case Direction::W:
      return p0.Pos().X() - p1.Pos().X();
      break;
  }
}


void Forward::initialize(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  start = mouse->pose;
  disp = 0.0f;
}

void Forward::execute(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  disp = forwardDisplacement(start,mouse->pose);

  float error = AbstractMaze::UNIT_DIST - disp;
  l = error * kP;
  r = error * kP;

  l = l > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : l;
  r = r > SimMouse::MAX_SPEED ? SimMouse::MAX_SPEED : r;

  l = l < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : l;
  r = r < SimMouse::MIN_SPEED ? SimMouse::MIN_SPEED : r;

  mouse->setSpeed(l,r);
}

bool Forward::isFinished(){
  printf("%f,%f %f\n", l, r, AbstractMaze::UNIT_DIST - disp);
  return disp > AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();
  mouse->setSpeed(0,0);
}
#endif
