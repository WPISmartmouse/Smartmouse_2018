#include "Mouse.h"
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include "AbstractMaze.h"
#include "Direction.h"

int Mouse::row = 0;
int Mouse::col = 0;
Direction Mouse::dir = Direction::S;

#ifdef SIM
std::mutex Mouse::pose_mutex;
std::condition_variable Mouse::pose_cond;
#endif

int Mouse::getRow(){
  return Mouse::row;
}

int Mouse::getCol(){
  return Mouse::col;
}

Direction Mouse::getDir(){
  return Mouse::dir;
}

bool Mouse::atCenter(){
	return (row == AbstractMaze::CENTER || row == AbstractMaze::CENTER-1) && (col == AbstractMaze::CENTER || col == AbstractMaze::CENTER-1);
}

bool Mouse::inBounds(){
  return row >= 0 && col >= 0
    && row < AbstractMaze::MAZE_SIZE - 1
    && col < AbstractMaze::MAZE_SIZE - 1;
}

void Mouse::internalTurnToFace(Direction dir) {
  Mouse::dir = dir;
}

void Mouse::internalForward(){
  switch(dir){
    case Direction::N: row--; break;
    case Direction::E: col++; break;
    case Direction::S: row++; break;
    case Direction::W: col--; break;
  }
}

#ifdef CONSOLE
int Mouse::forward(){
  internalForward();
	if (row >= AbstractMaze::MAZE_SIZE || row < 0 || col >= AbstractMaze::MAZE_SIZE || col < 0){
    //this is probably the most serious error possible
    //it means you've run into a wall. Just give up.
    printf("RAN INTO A FUCKING WALL\n");
    exit(-1);
	}
  return 0;
}

void Mouse::turn_to_face(char c){
  turn_to_face(char_to_dir(c));
}

void Mouse::turn_to_face(Direction d){
  if (d == Direction::INVALID){
    //again, this is a super serious error... you can't ever do this.
    printf("YOU CAN'T TURN THAT WAY\n");
    exit(-1);
  }
	if (dir != d){
    internalTurnToFace(d);
	}
	//in reality this will turn the physical mouse
}
#endif
#ifdef SIM
ignition::math::Pose3d Mouse::pose;
gazebo::transport::PublisherPtr Mouse::control_pub;

void Mouse::simInit(){
  gazebo::msgs::GzString stop_msg;
  stop_msg.set_data("stop");
  printf("Stop.\n");
  for (int i=0;i<10;i++) {
    control_pub->Publish(stop_msg);
    usleep(1000);
  }
  usleep(10000);
}

void Mouse::pose_callback(ConstPosePtr &msg){
  pose.Pos().X(msg->position().x());
  pose.Pos().Y(msg->position().y());
  pose.Pos().Z(msg->position().z());

  pose.Rot().X(msg->orientation().x());
  pose.Rot().Y(msg->orientation().y());
  pose.Rot().Z(msg->orientation().z());
  pose.Rot().W(msg->orientation().w());

  pose_cond.notify_all();
}

float Mouse::forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1){
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

int Mouse::forward(){
  printf("go forward\n");

  std::unique_lock<std::mutex> lk(pose_mutex);
  pose_cond.wait(lk);

  ignition::math::Pose3d start = pose;
  float disp;
  do {
    pose_cond.wait(lk);
    disp = forwardDisplacement(start,pose);

    gazebo::msgs::GzString msg;
    msg.set_data("forward");
    control_pub->Publish(msg);

    printf("disp=%f\n", disp);
  }
  while (disp < 0.168);
  internalForward();

  printf("Stop.\n");
  gazebo::msgs::GzString stop_msg;
  stop_msg.set_data("stop");
  control_pub->Publish(stop_msg);

  return 0;
}

void Mouse::turn_to_face(char c){
  turn_to_face(char_to_dir(c));
}


float Mouse::rotation(ignition::math::Pose3d p0,
            ignition::math::Pose3d p1){
  float y1 = p1.Rot().Yaw();
  float y0 = p0.Rot().Yaw();

  if (y0 < 0.0) { y0 += 2*M_PI; }
  if (y1 < 0.0) { y1 += 2*M_PI; }

  return y1 - y0;
}

float Mouse::absYawDiff(float y1, float y2){
  if (y2 < 0 && y1 > 0) { return fabs(y1 + y2); };
  if (y1 < 0 && y2 > 0) { return fabs(y1 + y2); };
  return fabs(y2-y1);
}

void Mouse::turn_to_face(Direction d){
  if (getDir() == d) {
    return;
  }

  std::unique_lock<std::mutex> lk(pose_mutex);
  pose_cond.wait(lk);

  ignition::math::Pose3d start = pose;
  float goalYaw = toYaw(d);
  float dYaw;
  int i=0;
  do {
    pose_cond.wait(lk);
    float currentYaw = pose.Rot().Yaw();
    dYaw = absYawDiff(currentYaw, goalYaw);

    gazebo::msgs::GzString turn_msg;
    turn_msg.set_data("turn cw");
    control_pub->Publish(turn_msg);

    printf("yaw=%f goal=%f dYaw=%f d=%c\n",
        currentYaw, goalYaw, dYaw, dir_to_char(d));
  }
  while (dYaw > ROT_TOLERANCE);
  internalTurnToFace(d);

  printf("Stop.\n");
  gazebo::msgs::GzString stop_msg;
  stop_msg.set_data("stop");
  control_pub->Publish(stop_msg);
}
#endif
#ifdef EMBED
int Mouse::forward(){
  //#TODO ACTUAL MOUSE MOVE CODE HERE
  return 0;
}

void Mouse::turn_to_face(char c){
  turn_to_face(char_to_dir(c));
}

void Mouse::turn_to_face(Direction d){
  //#TODO ACTUAL MOUSE MOVE CODE HERE
}
#endif
