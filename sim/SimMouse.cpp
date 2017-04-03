#include "SimMouse.h"
#include <gazebo/msgs/msgs.hh>

SimMouse *SimMouse::instance = nullptr;
const gazebo::common::Color SimMouse::grey_color{0.8, 0.8, 0.8, 1};
RobotConfig SimMouse::CONFIG;

double SimMouse::abstractForceToNewtons(double x) {
  // abstract force is from -255 to 255 per motor
  return x * MAX_FORCE / 255.0;
}

SimMouse::SimMouse() {
  CONFIG.ANALOG_ANGLE = 1.35255; // radians
  CONFIG.SIDE_ANALOG_X = 0.04; // meters
  CONFIG.SIDE_ANALOG_Y = 0.024; // meters
  CONFIG.MAX_SPEED = 0.09; // m/s
  CONFIG.MIN_SPEED = 0.005; // m/s
  CONFIG.WALL_DIST = 0.125; // meters
  CONFIG.BINARY_ANGLE = 0.65; // radians
}

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

  sr.walls[static_cast<int>(dir)] = range_data.front_binary;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data.left_binary;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data.right_binary;
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

double SimMouse::getColOffsetToEdge() {
  return this->col_offset_to_edge;
}

int SimMouse::getComputedCol() {
  return this->computed_col;
}

int SimMouse::getComputedRow() {
  return this->computed_row;
}

Pose SimMouse::getEstimatedPose() {
  Pose pose = kinematic_controller.getPose();
  return pose;
}

Pose SimMouse::getExactPose() {
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return true_pose;
}

RangeData SimMouse::getRangeData() {
  //wait for the next message to occur
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);
  return this->range_data;
}

double SimMouse::getRowOffsetToEdge() {
  return this->row_offset_to_edge;
}

std::pair<double, double> SimMouse::getWheelVelocities() {
  return kinematic_controller.getWheelVelocities();
};

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

bool SimMouse::isStopped() {
  return kinematic_controller.isStopped() && fabs(abstract_left_force) <= 5 &&
         fabs(abstract_right_force) <= RegulatedMotor::MIN_ABSTRACT_FORCE;
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

  this->left_wheel_velocity_mps = msg->left_wheel_velocity_mps();
  this->right_wheel_velocity_mps = msg->right_wheel_velocity_mps();

  this->left_wheel_angle_rad = msg->left_wheel_angle_radians();
  this->right_wheel_angle_rad = msg->right_wheel_angle_radians();

  this->range_data.left_analog = msg->left_analog();
  this->range_data.right_analog = msg->right_analog();
  this->range_data.left_binary = msg->left_binary();
  this->range_data.right_binary = msg->right_binary();
  this->range_data.front_binary = msg->front_binary();

  dataCond.notify_all();
}

void SimMouse::run(unsigned long time_ms) {
  // handle updating of odometry and PID
  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(time_ms,
                                                                                 this->left_wheel_angle_rad,
                                                                                 this->right_wheel_angle_rad,
                                                                                 this->left_wheel_velocity_mps,
                                                                                 this->right_wheel_velocity_mps);

  // update row/col information
  Pose estimated_pose = kinematic_controller.getPose();
  computed_row = (int) (estimated_pose.y / AbstractMaze::UNIT_DIST);
  computed_col = (int) (estimated_pose.x / AbstractMaze::UNIT_DIST);

  row_offset_to_edge = fmod(estimated_pose.y, AbstractMaze::UNIT_DIST);
  col_offset_to_edge = fmod(estimated_pose.x, AbstractMaze::UNIT_DIST);

  // not sure if this is a good idea
  row = computed_row;
  col = computed_col;

  // publish status information
  gzmaze::msgs::MazeLocation maze_loc_msg;
  maze_loc_msg.set_row(getComputedRow());
  maze_loc_msg.set_col(getComputedCol());
  maze_loc_msg.set_row_offset(getRowOffsetToEdge());
  maze_loc_msg.set_col_offset(getColOffsetToEdge());

  maze_loc_msg.set_estimated_x_meters(estimated_pose.x);
  maze_loc_msg.set_estimated_y_meters(estimated_pose.y);
  maze_loc_msg.set_estimated_yaw_rad(estimated_pose.yaw);
  std::string dir_str(1, dir_to_char(dir));
  maze_loc_msg.set_dir(dir_str);

  if (!maze_loc_msg.IsInitialized()) {
    std::cerr << "Missing fields: [" << maze_loc_msg.InitializationErrorString() << "]" << std::endl;
  }

  maze_location_pub->Publish(maze_loc_msg);

  double left_force_newtons = abstractForceToNewtons(abstract_left_force);
  gazebo::msgs::JointCmd left;
  left.set_name("mouse::left_wheel_joint");
  left.set_force(left_force_newtons);
  joint_cmd_pub->Publish(left);

  double right_force_newtons = abstractForceToNewtons(abstract_right_force);
  gazebo::msgs::JointCmd right;
  right.set_name("mouse::right_wheel_joint");
  right.set_force(right_force_newtons);
  joint_cmd_pub->Publish(right);
}

void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps) {
  kinematic_controller.setSpeedMps(left_wheel_velocity_setpoint_mps, right_wheel_velocity_setpoint_mps);
}

void SimMouse::simInit() {
  setSpeed(0, 0);

  // we start in the middle of the first square
  kinematic_controller.reset_x_to(AbstractMaze::HALF_UNIT_DIST);
  kinematic_controller.reset_y_to(0.05);
  kinematic_controller.setAcceleration(1, 2);

//  for (int i = 0; i < AbstractMaze::MAZE_SIZE; i++) { for (int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
//      indicators[i][j] = new gazebo::msgs::Visual();
//      updateIndicator(i, j, grey_color);
//    }
//  }
//  publishIndicators();
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

