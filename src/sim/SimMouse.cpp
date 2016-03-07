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

float SimMouse::forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1){
  switch(getDir()){
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

int SimMouse::forward(){
  printf("go forward\n");

  std::unique_lock<std::mutex> lk(poseMutex);
  poseCond.wait(lk);

  ignition::math::Pose3d start = pose;
  float disp;
  do {
    poseCond.wait(lk);
    disp = forwardDisplacement(start,pose);

    gazebo::msgs::Vector2d msg;
    msg.set_x(0);
    msg.set_y(0);
    controlPub->Publish(msg);

    printf("disp=%f\n", disp);
  }
  while (disp < 0.168);
  internalForward();

  printf("Stop.\n");
  gazebo::msgs::Vector2d stop_msg;
  stop_msg.set_x(0);
  stop_msg.set_y(0);
  controlPub->Publish(stop_msg);

  return 0;
}

float SimMouse::rotation(ignition::math::Pose3d p0,
            ignition::math::Pose3d p1){
  float y1 = p1.Rot().Yaw();
  float y0 = p0.Rot().Yaw();

  if (y0 < 0.0) { y0 += 2*M_PI; }
  if (y1 < 0.0) { y1 += 2*M_PI; }

  return y1 - y0;
}

float SimMouse::absYawDiff(float y1, float y2){
  float diff = y2 - y1;
  if (diff > 180) return fabs(diff - 360);
  if (diff < -180) return fabs(diff + 360);
  return fabs(diff);
}

void SimMouse::turnToFace(Direction d){
  if (getDir() == d) {
    return;
  }

  std::unique_lock<std::mutex> lk(poseMutex);
  poseCond.wait(lk);

  ignition::math::Pose3d start = pose;
  float goalYaw = toYaw(d);
  float dYaw;
  int i=0;
  do {
    poseCond.wait(lk);
    float currentYaw = pose.Rot().Yaw();
    dYaw = absYawDiff(currentYaw, goalYaw);

    gazebo::msgs::Vector2d turn_msg;
    turn_msg.set_x(0.01);
    turn_msg.set_y(0.01);
    controlPub->Publish(turn_msg);

    printf("yaw=%f goal=%f dYaw=%f d=%c\n",
        currentYaw, goalYaw, dYaw, dir_to_char(d));
  }
  while (dYaw > ROT_TOLERANCE);
  internalTurnToFace(d);

  printf("Stop.\n");
  gazebo::msgs::Vector2d stop_msg;
  stop_msg.set_x(0);
  stop_msg.set_y(0);
  controlPub->Publish(stop_msg);
}
#endif
