#ifdef SIM
#include "SimMouse.h"
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>

void SimMouse::simInit(){
  gazebo::msgs::Vector2d stop_msg;
  stop_msg.set_x(0);
  stop_msg.set_y(0);
  printf("Stop.\n");
  for (int i=0;i<10;i++) {
    controlPub->Publish(stop_msg);
    usleep(1000);
  }
  usleep(10000);
}

void SimMouse::poseCallback(ConstPosePtr &msg){
  pose.Pos().X(msg->position().x());
  pose.Pos().Y(msg->position().y());
  pose.Pos().Z(msg->position().z());

  pose.Rot().X(msg->orientation().x());
  pose.Rot().Y(msg->orientation().y());
  pose.Rot().Z(msg->orientation().z());
  pose.Rot().W(msg->orientation().w());

  poseCond.notify_all();
}
#endif
