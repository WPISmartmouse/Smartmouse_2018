#include "Mouse.h"
#include <stdio.h>
#include "AbstractMaze.h"

int Mouse::row = 0;
int Mouse::col = 0;
Direction Mouse::dir = Direction::S;

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

#ifdef CONSOLE
int Mouse::forward(){
  switch(dir){
    case Direction::N:
      row--;
      break;
    case Direction::E:
      col++;
      break;
    case Direction::S:
      row++;
      break;
    case Direction::W:
      col--;
      break;
    default:
      return -1;
  }

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
    printf("YOU CAN'T TURNTHAT WAY\n");
    exit(-1);
  }
	if (dir != d){
		dir = d;
	}
	//in reality this will turn the physical mouse
}
#endif
#ifdef SIM
ignition::math::Pose3d Mouse::pose;
gazebo::transport::PublisherPtr Mouse::control_pub;

void Mouse::pose_callback(ConstPosePtr &msg){
  pose.Pos().X(msg->position().x());
  pose.Pos().Y(msg->position().y());
  pose.Pos().Z(msg->position().z());

  pose.Rot().X(msg->orientation().x());
  pose.Rot().Y(msg->orientation().y());
  pose.Rot().Z(msg->orientation().z());
  pose.Rot().W(msg->orientation().w());
}

double Mouse::forwardDisplacement(ignition::math::Pose3d p0, ignition::math::Pose3d p1){
  ignition::math::Vector3d b = p1.Pos() - p0.Pos();
  ignition::math::Vector3d a;

  switch(getDir()){
    case Direction::N:
      a = ignition::math::Vector3d(0,1,0);
      break;
    case Direction::E:
      a = ignition::math::Vector3d(1,0,0);
      break;
    case Direction::S:
      a = ignition::math::Vector3d(0,-1,0);
      break;
    case Direction::W:
      a = ignition::math::Vector3d(-1,0,0);
      break;
  }

  //projection of b unto a
  return a.Dot(b)/a.Length();
}

int Mouse::forward(){
  gazebo::msgs::GzString msg;
  msg.set_data("forward");
  control_pub->Publish(msg);

  ignition::math::Pose3d start = pose;
  while (forwardDisplacement(start, pose) < 0.168) {
    //wait until we've reached our location
  }
  printf("finished moving\n");

  return 0;
}

void Mouse::turn_to_face(char c){
  gazebo::msgs::GzString msg;
  std::string command = "turn to face ";
  command += c;
  msg.set_data(command);
  control_pub->Publish(msg);
}

void Mouse::turn_to_face(Direction d){
  turn_to_face(dir_to_char(d));
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
