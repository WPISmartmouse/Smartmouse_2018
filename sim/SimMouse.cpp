#ifdef SIM

#include "SimMouse.h"
#include <gazebo/msgs/msgs.hh>

SimMouse *SimMouse::instance = nullptr;
const gazebo::common::Color SimMouse::grey_color{0.8, 0.8, 0.8, 1};

double SimMouse::abstractForceToNewtons(double x) {
  // abstract force is from -255 to 255 per motor
  return x * MAX_FORCE / 255.0;
}

SimMouse::SimMouse() : hasSuggestion(false), kinematic_controller(CONTROL_PERIOD_MS) {}

SimMouse *SimMouse::inst() {
  if (instance == NULL) {
    instance = new SimMouse();
  }

  return instance;
}

SensorReading SimMouse::checkWalls() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  SensorReading sr(row, col);
  std::array<bool, 4> *w = &sr.walls;

  for (int i = 0; i < w->size(); i++) {
    if (hasSuggestion) {
      (*w)[i] = suggestedWalls[i];
    } else {
      (*w)[i] = this->walls[i];
    }
  }

  if (!hasSuggestion) {
    (*w)[0] = true; // there will always be a wall North
  }

  return sr;
}

double SimMouse::getColOffsetToEdge() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->col_offset_to_edge;
}

int SimMouse::getComputedCol() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->computed_col;
}

int SimMouse::getComputedRow() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->computed_row;
}

Pose SimMouse::getEstimatedPose() {
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  Pose pose = kinematic_controller.get_pose();
  return pose;
}

Pose SimMouse::getExactPose() {
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return true_pose;
}

SimMouse::RangeData SimMouse::getRangeData() {
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->range_data;
}

double SimMouse::getRowOffsetToEdge() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->row_offset_to_edge;
}

std::pair<double, double> SimMouse::getWheelVelocities() {
  std::pair<double, double> pair;
  pair.first = radPerSecToMetersPerSec(this->left_wheel_velocity);
  pair.second = radPerSecToMetersPerSec(this->right_wheel_velocity);
  return pair;
}

void SimMouse::indicatePath(int row, int col, std::string path, gazebo::common::Color color) {
  for (char &c : path) {
    switch (c) {
      case 'N':
        row--;
        break;
      case 'E':
        col++;
        break;
      case 'S':
        row++;
        break;
      case 'W':
        col--;
        break;
      default:
        break;
    }
    updateIndicator(row, col, color);
  }
  publishIndicators();
}

void SimMouse::publishIndicators() {
  for (int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      indicator_pub->Publish(*indicators[i][j]);
    }
  }
}

void SimMouse::resetIndicators(gazebo::common::Color color) {
  for (int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      gazebo::msgs::Color c = indicators[i][j]->material().diffuse();
      if (color.r == c.r()
          & color.g == c.g()
          & color.b == c.b()
          & color.a == c.a()) {
        updateIndicator(i, j, grey_color);
      }
    }
  }
}

void SimMouse::robotStateCallback(ConstRobotStatePtr &msg) {
  true_pose.x = msg->true_x_meters();
  true_pose.y = msg->true_y_meters();
  true_pose.yaw = msg->true_yaw_rad();

  // TODO: Consider also computing this on estimate pose?
  computed_row = (int) (msg->true_y_meters() / AbstractMaze::UNIT_DIST);
  computed_col = (int) (msg->true_x_meters() / AbstractMaze::UNIT_DIST);
  row_offset_to_edge = fmod(msg->true_y_meters(), AbstractMaze::UNIT_DIST);
  col_offset_to_edge = fmod(msg->true_x_meters(), AbstractMaze::UNIT_DIST);

  this->left_wheel_velocity = msg->left_wheel_velocity_mps();
  this->right_wheel_velocity = msg->right_wheel_velocity_mps();

  this->left_wheel_angle_rad = msg->left_wheel_angle_radians();
  this->right_wheel_angle_rad = msg->right_wheel_angle_radians();

  //transform from Mouse frame to Cardinal frame;
  this->range_data.left_analog = msg->left_analog();
  this->range_data.right_analog = msg->right_analog();
  this->range_data.left_binary = msg->left_binary();
  this->range_data.right_binary = msg->right_binary();
  this->range_data.front_binary = msg->front_binary();

  switch (dir) {
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

void SimMouse::run(unsigned long time_ms) {
  // publish status information
  gzmaze::msgs::MazeLocation msg;
  msg.set_row(getComputedRow());
  msg.set_col(getComputedCol());
  msg.set_row_offset(getRowOffsetToEdge());
  msg.set_col_offset(getColOffsetToEdge());

  Pose estimated_pose = getEstimatedPose();
  msg.set_estimated_x_meters(estimated_pose.x);
  msg.set_estimated_y_meters(estimated_pose.y);
  msg.set_estimated_yaw_rad(estimated_pose.yaw);
  std::string dir_str(1, dir_to_char(getDir()));
  msg.set_dir(dir_str);

  if (!msg.IsInitialized()) {
    std::cerr << "Missing fields: [" << msg.InitializationErrorString() << "]" << std::endl;
  }

  maze_location_pub->Publish(msg);

  // handle updating of odometry and PID
  double abstract_left_force;
  double abstract_right_force;

  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(time_ms, this->left_wheel_angle_rad,
                                                                                 this->right_wheel_angle_rad);

  double left_force_newtons = abstractForceToNewtons(abstract_left_force);
  gazebo::msgs::JointCmd left;
  left.set_name("mouse::left_wheel_joint");
  left.set_force(left_force_newtons);
//  joint_cmd_pub->Publish(left);

  double right_force_newtons = abstractForceToNewtons(abstract_right_force);
  gazebo::msgs::JointCmd right;
  right.set_name("mouse::right_wheel_joint");
  right.set_force(right_force_newtons);
//  joint_cmd_pub->Publish(right);
}

void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps) {
  kinematic_controller.setSpeed(left_wheel_velocity_setpoint_mps, right_wheel_velocity_setpoint_mps);
}

void SimMouse::simInit() {
  setSpeed(0, 0);

  // we start in the middle of the first square
  kinematic_controller.reset_x_to(AbstractMaze::HALF_UNIT_DIST);
  kinematic_controller.setAcceleration(START_ACCELERATION, STOP_ACCELERATION);

  for (int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      indicators[i][j] = new gazebo::msgs::Visual();
      updateIndicator(i, j, grey_color);
    }
  }
  publishIndicators();
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

  double zero_offset = (AbstractMaze::UNIT_DIST * (AbstractMaze::MAZE_SIZE - 1) / 2);
  double y = zero_offset - row * AbstractMaze::UNIT_DIST;
  double x = -zero_offset + col * AbstractMaze::UNIT_DIST;

  gazebo::msgs::Set(visual->mutable_pose(),
                    ignition::math::Pose3d(x, y, INDICATOR_Z, 0, 0, 0));

  gazebo::msgs::Set(visual->mutable_material()->mutable_diffuse(), color);
}

#endif
