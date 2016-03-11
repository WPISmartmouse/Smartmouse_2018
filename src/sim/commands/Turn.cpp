#ifdef SIM

#include "Turn.h"

#include <gazebo/msgs/msgs.hh>

Turn::Turn(Mouse *mouse, Direction dir) : mouse((SimMouse *)mouse), dir(dir) {}

void Turn::initialize(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  ignition::math::Pose3d start = mouse->pose;
  goalYaw = toYaw(dir);
}

float Turn::absYawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > 180) return fabs(diff - 360);
  if (diff < -180) return fabs(diff + 360);
  return fabs(diff);
}

void Turn::execute(){
  std::unique_lock<std::mutex> lk(mouse->poseMutex);
  mouse->poseCond.wait(lk);
  float currentYaw = mouse->pose.Rot().Yaw();
  dYaw = absYawDiff(currentYaw, goalYaw);

  gazebo::msgs::Vector2d turn_msg;
  turn_msg.set_x(0.01);
  turn_msg.set_y(0.01);
  mouse->controlPub->Publish(turn_msg);

}

bool Turn::isFinished(){
  return (mouse->getDir() == dir) || (dYaw > Mouse::ROT_TOLERANCE);
}

void Turn::end(){
  gazebo::msgs::Vector2d stop_msg;
  stop_msg.set_x(0);
  stop_msg.set_y(0);
  mouse->controlPub->Publish(stop_msg);
}
#endif
