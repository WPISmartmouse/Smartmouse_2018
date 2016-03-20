#ifdef SIM
#include "SimMouse.h"
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>

const float SimMouse::MAX_SPEED = 100;
const float SimMouse::MIN_SPEED = 20;
const float SimMouse::WALL_DIST = 0.12;
SimMouse *SimMouse::instance = nullptr;

const gazebo::common::Color SimMouse::red_color{1, 0, 0, 1};
const gazebo::common::Color SimMouse::green_color{0, 1, 0, 1};
const gazebo::common::Color SimMouse::blue_color{0, 0, 1, 1};
const gazebo::common::Color SimMouse::black_color{0, 0, 0, 1};
const gazebo::common::Color SimMouse::grey_color{0.8, 0.8, 0.8, 1};

SimMouse::SimMouse() : hasSuggestion(false) {}

SimMouse *SimMouse::inst(){
  if (instance == NULL){
    instance = new SimMouse();
  }

  return instance;
}

void SimMouse::updateIndicator(int row, int col, gazebo::common::Color color) {
  gazebo::msgs::Visual *visual = indicators[row][col];

  std::string visual_name = "my_maze::base::indicator_"
				+ std::to_string(row)
				+ "_" + std::to_string(col);
  visual->set_name(visual_name);
  visual->set_visible(true);
  visual->set_parent_name("my_maze::base");
  visual->set_cast_shadows(false);

	gazebo::msgs::Geometry *geomMsg = visual->mutable_geometry();
  geomMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
  geomMsg->mutable_cylinder()->set_radius(INDICATOR_RAD);
  geomMsg->mutable_cylinder()->set_length(INDICATOR_LEN);

  float zero_offset = (AbstractMaze::UNIT_DIST * (AbstractMaze::MAZE_SIZE - 1)/2);
  float y = zero_offset - row * AbstractMaze::UNIT_DIST;
  float x = -zero_offset + col * AbstractMaze::UNIT_DIST;

	gazebo::msgs::Set(visual->mutable_pose(),
      ignition::math::Pose3d(x, y, 0.02, 0, 0, 0));

  gazebo::msgs::Set(visual->mutable_material()->mutable_diffuse(), color);
}

void SimMouse::resetIndicators() {
  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
      updateIndicator(i,j,grey_color);
    }
  }
}

void SimMouse::publishIndicators() {
  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
      indicatorPub->Publish(*indicators[i][j]);
    }
  }
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

void SimMouse::checkWallsCallback(ConstLaserScanStampedPtr &msg){
  //transform from Mouse frame to Cardinal frame;
  int size = msg->scan().ranges_size();

  for (int i=0;i<size;i++){
    rawDistances[i] = msg->scan().ranges(i);
  }

  const int FRONT_INDEX = 1;
  const int RIGHT_INDEX = 2;

  switch(dir) {
    case Direction::N:
      walls[0] = msg->scan().ranges(FRONT_INDEX) < WALL_DIST;
      walls[1] = msg->scan().ranges(0) < WALL_DIST;
      walls[2] = false;
      walls[3] = msg->scan().ranges(RIGHT_INDEX) < WALL_DIST;
      break;
    case Direction::E:
      walls[0] = msg->scan().ranges(RIGHT_INDEX) < WALL_DIST;
      walls[1] = msg->scan().ranges(FRONT_INDEX) < WALL_DIST;
      walls[2] = msg->scan().ranges(0) < WALL_DIST;
      walls[3] = false;
      break;
    case Direction::S:
      walls[0] = false;
      walls[1] = msg->scan().ranges(RIGHT_INDEX) < WALL_DIST;
      walls[2] = msg->scan().ranges(FRONT_INDEX) < WALL_DIST;
      walls[3] = msg->scan().ranges(0) < WALL_DIST;
      break;
    case Direction::W:
      walls[0] = msg->scan().ranges(0) < WALL_DIST;
      walls[1] = false;
      walls[2] = msg->scan().ranges(RIGHT_INDEX) < WALL_DIST;
      walls[3] = msg->scan().ranges(FRONT_INDEX) < WALL_DIST;;
      break;
  }
  checkWallsCond.notify_all();
}

SensorReading SimMouse::checkWalls(){
  std::unique_lock<std::mutex> lk(checkWallsMutex);
  checkWallsCond.wait(lk);
  SensorReading sr(row, col);
  std::array<bool, 4> *w = &sr.walls;

  for (int i=0;i<w->size();i++){
    if (hasSuggestion){
      (*w)[i] = suggestedWalls[i];
    }
    else {
      (*w)[i] = this->walls[i];
    }
  }

  return sr;
}

void SimMouse::suggestWalls(bool *walls) {
  hasSuggestion = true;
  for (int i=0;i<4;i++){
    suggestedWalls[i] = walls[i];
  }
}

void SimMouse::simInit(){
  setSpeed(0,0);
  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
      indicators[i][j] = new gazebo::msgs::Visual();
      updateIndicator(i,j, grey_color);
    }
  }
  publishIndicators();
}

//lspeed and rspeed should be from -1 to 1
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

float *SimMouse::getRawDistances(){
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(checkWallsMutex);
  checkWallsCond.wait(lk);
  return rawDistances;
}

ignition::math::Pose3d SimMouse::getPose(){
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(poseMutex);
  poseCond.wait(lk);
  return pose;
}

gazebo::msgs::Pose *SimMouse::createPose(int row, int col, float z) {
  float zero_offset = (AbstractMaze::UNIT_DIST * (AbstractMaze::MAZE_SIZE - 1)/2);
  float x = -zero_offset + col * AbstractMaze::UNIT_DIST;
  float y = zero_offset - row * AbstractMaze::UNIT_DIST;

  gazebo::msgs::Vector3d *position = new gazebo::msgs::Vector3d();
  position->set_x(x);
  position->set_y(y);
  position->set_z(z);

  gazebo::msgs::Quaternion *orientation = new gazebo::msgs::Quaternion();
  orientation->set_z(0);
  orientation->set_w(1);

  gazebo::msgs::Pose *pose = new gazebo::msgs::Pose;
  pose->set_allocated_orientation(orientation);
  pose->set_allocated_position(position);
}

gazebo::msgs::Geometry *SimMouse::createCylinderGeometry(float r, float l)
{
  gazebo::msgs::CylinderGeom *cylinder = new gazebo::msgs::CylinderGeom();
  cylinder->set_radius(r);
  cylinder->set_length(l);

  gazebo::msgs::Geometry *geo = new gazebo::msgs::Geometry();
  geo->set_type(gazebo::msgs::Geometry_Type_CYLINDER);
  geo->set_allocated_cylinder(cylinder);

  return geo;
}

#endif
