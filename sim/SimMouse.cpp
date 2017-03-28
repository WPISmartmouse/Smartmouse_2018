#ifdef SIM
#include "SimMouse.h"
#include <gazebo/msgs/msgs.hh>

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

  gazebo::msgs::Visual::Meta *meta = visual->mutable_meta();
  meta->set_layer(2);

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

  double zero_offset = (AbstractMaze::UNIT_DIST * (AbstractMaze::MAZE_SIZE - 1)/2);
  double y = zero_offset - row * AbstractMaze::UNIT_DIST;
  double x = -zero_offset + col * AbstractMaze::UNIT_DIST;

	gazebo::msgs::Set(visual->mutable_pose(),
      ignition::math::Pose3d(x, y, INDICATOR_Z, 0, 0, 0));

  gazebo::msgs::Set(visual->mutable_material()->mutable_diffuse(), color);
}

void SimMouse::resetIndicators(gazebo::common::Color color) {
  for (int i=0;i<AbstractMaze::MAZE_SIZE;i++){
    for (int j=0;j<AbstractMaze::MAZE_SIZE;j++){
      gazebo::msgs::Color c = indicators[i][j]->material().diffuse();
      if (color.r == c.r()
          &color.g == c.g()
          &color.b == c.b()
          &color.a == c.a()) {
        updateIndicator(i,j,grey_color);
      }
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

void SimMouse::indicatePath(int row, int col, std::string path, gazebo::common::Color color) {
  for(char& c : path) {
    switch(c){
      case 'N': row--; break;
      case 'E': col++; break;
      case 'S': row++; break;
      case 'W': col--; break;
      default: break;
    }
    updateIndicator(row, col, color);
  }
  publishIndicators();
}

void SimMouse::robotStateCallback(ConstRobotStatePtr &msg){
  pose.Pos().X(msg->pose().position().x());
  pose.Pos().Y(msg->pose().position().y());
  pose.Pos().Z(msg->pose().position().z());

  pose.Rot().X(msg->pose().orientation().x());
  pose.Rot().Y(msg->pose().orientation().y());
  pose.Rot().Z(msg->pose().orientation().z());
  pose.Rot().W(msg->pose().orientation().w());

  double x = pose.Pos().X() + AbstractMaze::UNIT_DIST * 8;
  double y = -pose.Pos().Y() + AbstractMaze::UNIT_DIST * 8;

  computed_row = (int) (y / AbstractMaze::UNIT_DIST);
  computed_col = (int) (x / AbstractMaze::UNIT_DIST);
  row_offset_to_edge = fmod(y, AbstractMaze::UNIT_DIST);
  col_offset_to_edge = fmod(x, AbstractMaze::UNIT_DIST);

  this->left_wheel_velocity = msg->left_wheel_velocity_mps();
  this->right_wheel_velocity = msg->right_wheel_velocity_mps();

  this->left_wheel_angle = msg->left_wheel_angle_radians();
  this->right_wheel_angle = msg->right_wheel_angle_radians();

  //transform from Mouse frame to Cardinal frame;
  this->range_data.left_analog = msg->left_analog();
  this->range_data.right_analog = msg->right_analog();
  this->range_data.left_binary = msg->left_binary();
  this->range_data.right_binary = msg->right_binary();
  this->range_data.front_binary = msg->front_binary();

  switch(dir) {
    case Direction::N:
      walls[0] = range_data.front_binary;
      walls[1] = range_data.left_binary;
      walls[2] = false;
      walls[3] = range_data.right_binary;
      break;
    case Direction::E:
      walls[0] = range_data.right_binary;
      walls[1] = range_data.front_binary;
      walls[2] = range_data.left_binary;
      walls[3] = false;
      break;
    case Direction::S:
      walls[0] = false;
      walls[1] = range_data.right_binary;
      walls[2] = range_data.front_binary;
      walls[3] = range_data.left_binary;
      break;
    case Direction::W:
      walls[0] = range_data.left_binary;
      walls[1] = false;
      walls[2] = range_data.right_binary;
      walls[3] = range_data.front_binary;
      break;
  }
  dataCond.notify_all();
}

SensorReading SimMouse::checkWalls(){
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
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

  if (!hasSuggestion){
    (*w)[0] = true; // there will always be a wall North
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
void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps){
  static double left_wheel_velocity_mps;
  static double right_wheel_velocity_mps;

  double left_acc = FWD_ACCELERATION;
  double right_acc = FWD_ACCELERATION;
  if (left_wheel_velocity_setpoint_mps == 0) {
    left_acc = STOP_ACCELERATION;
  }
  if (right_wheel_velocity_setpoint_mps == 0) {
    right_acc = STOP_ACCELERATION;
  }

  if (right_wheel_velocity_mps < right_wheel_velocity_setpoint_mps) {
    right_wheel_velocity_mps = std::min(right_wheel_velocity_mps + right_acc, right_wheel_velocity_setpoint_mps);
  } else if (right_wheel_velocity_mps > right_wheel_velocity_setpoint_mps) {
    right_wheel_velocity_mps = std::max(right_wheel_velocity_mps - right_acc, right_wheel_velocity_setpoint_mps);
  }

  if (left_wheel_velocity_mps < left_wheel_velocity_setpoint_mps) {
    left_wheel_velocity_mps = std::min(left_wheel_velocity_mps + left_acc, left_wheel_velocity_setpoint_mps);
  } else if (left_wheel_velocity_mps > left_wheel_velocity_setpoint_mps) {
    left_wheel_velocity_mps = std::max(left_wheel_velocity_mps - left_acc, left_wheel_velocity_setpoint_mps);
  }

  double left_wheel_velocity = metersPerSecToRadPerSec(left_wheel_velocity_mps);
  double right_wheel_velocity = metersPerSecToRadPerSec(right_wheel_velocity_mps);

  gazebo::msgs::JointCmd left;
	left.set_name("mouse::left_wheel_joint");
	left.mutable_velocity()->set_target(left_wheel_velocity);
	left.mutable_velocity()->set_p_gain(kP);
	left.mutable_velocity()->set_i_gain(kI);
	left.mutable_velocity()->set_d_gain(kD);
	controlPub->Publish(left);

  gazebo::msgs::JointCmd right;
	right.set_name("mouse::right_wheel_joint");
	right.mutable_velocity()->set_target(right_wheel_velocity);
	right.mutable_velocity()->set_p_gain(kP);
	right.mutable_velocity()->set_i_gain(kI);
	right.mutable_velocity()->set_d_gain(kD);
	controlPub->Publish(right);
}

SimMouse::RangeData SimMouse::getRangeData(){
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->range_data;
}

Pose SimMouse::getEstimatedPose() {
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  Pose pose;
  pose.x = 0;
  pose.y = 0;
  pose.yaw = 0;
  return pose;
}

ignition::math::Pose3d SimMouse::getExactPose(){
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return pose;
}

double SimMouse::metersPerSecToRadPerSec(double x) {
  return x / WHEEL_CIRC * (2 * M_PI);
}

double SimMouse::radPerSecToMetersPerSec(double x) {
  return (x / (2 * M_PI)) * WHEEL_CIRC;
}

std::pair<double, double> SimMouse::getWheelVelocities() {
  std::pair<double, double> pair;
  pair.first = radPerSecToMetersPerSec(this->left_wheel_velocity);
  pair.second = radPerSecToMetersPerSec(this->right_wheel_velocity);
  return pair;
}

int SimMouse::getComputedCol() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return  this->computed_col;
}

int SimMouse::getComputedRow() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return  this->computed_row;
}

double SimMouse::getRowOffsetToEdge() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return  this->row_offset_to_edge;
}

double SimMouse::getColOffsetToEdge() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return  this->col_offset_to_edge;
}
#endif
