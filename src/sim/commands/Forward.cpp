#ifdef SIM

#include "Forward.h"
#include "Direction.h"
#include "AbstractMaze.h"

Forward::Forward(Mouse *mouse) : mouse((SimMouse *)mouse) {}

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
  ignition::math::Pose3d start = mouse->pose;
  disp = 0.0f;
}

void Forward::execute(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  disp = forwardDisplacement(start,mouse->pose);

  gazebo::msgs::Vector2d msg;
  msg.set_x(0);
  msg.set_y(0);
  mouse->controlPub->Publish(msg);
}

bool Forward::isFinished(){
  return disp < AbstractMaze::UNIT_DIST;
}

void Forward::end(){
  mouse->internalForward();

  gazebo::msgs::Vector2d stop_msg;
  stop_msg.set_x(0);
  stop_msg.set_y(0);
  mouse->controlPub->Publish(stop_msg);
}
#endif
