#ifdef SIM
#include <gazebo/msgs/msgs.hh>
#include "SimMouse.h"
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>

SimMouse::SimMouse(AbstractMaze *maze) : Mouse(maze) {}

void SimMouse::senseCallback(ConstLaserScanStampedPtr &msg){
  //transform from Mouse frame to Cardinal frame;
  int size = msg->scan().ranges_size();

  for (int i=0;i<size;i++){
    rawDistances[i] = msg->scan().ranges(i);
  }

  switch(dir) {
    case Direction::N:
      walls[0] = msg->scan().ranges(2) < WALL_DIST;
      walls[1] = msg->scan().ranges(0) < WALL_DIST;
      walls[2] = false;
      walls[3] = msg->scan().ranges(4) < WALL_DIST;
      break;
    case Direction::E:
      walls[0] = msg->scan().ranges(4) < WALL_DIST;
      walls[1] = msg->scan().ranges(2) < WALL_DIST;
      walls[2] = msg->scan().ranges(0) < WALL_DIST;
      walls[3] = false;
      break;
    case Direction::S:
      walls[0] = false;
      walls[1] = msg->scan().ranges(4) < WALL_DIST;
      walls[2] = msg->scan().ranges(2) < WALL_DIST;
      walls[3] = msg->scan().ranges(0) < WALL_DIST;
      break;
    case Direction::W:
      walls[0] = msg->scan().ranges(0) < WALL_DIST;
      walls[1] = false;
      walls[2] = msg->scan().ranges(4) < WALL_DIST;
      walls[3] = msg->scan().ranges(2) < WALL_DIST;;
      break;
  }
  senseCond.notify_all();
}

SensorReading SimMouse::sense(){
  std::unique_lock<std::mutex> lk(senseMutex);
  senseCond.wait(lk);
  SensorReading sr(row, col);
  bool *w = sr.walls;
  for (int i=0;i<4;i++){
    *(w++) = walls[i];
  }

  return sr;
}

void SimMouse::simInit(){
  setSpeed(0,0);
}

void SimMouse::setSpeed(float lspeed, float rspeed){
  gazebo::msgs::JointCmd left;
	left.set_name("mouse::left_wheel_joint");
	left.mutable_velocity()->set_target(lspeed);
	left.mutable_velocity()->set_p_gain(kP);
	left.mutable_velocity()->set_i_gain(kI);
	left.mutable_velocity()->set_d_gain(kD);
	controlPub->Publish(left);

  gazebo::msgs::JointCmd right;
	right.set_name("mouse::right_wheel_joint");
	right.mutable_velocity()->set_target(rspeed);
	right.mutable_velocity()->set_p_gain(kP);
	right.mutable_velocity()->set_i_gain(kI);
	right.mutable_velocity()->set_d_gain(kD);
	controlPub->Publish(right);
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

ignition::math::Pose3d SimMouse::getPose(){
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(poseMutex);
  poseCond.wait(lk);
  return pose;
}
#endif
