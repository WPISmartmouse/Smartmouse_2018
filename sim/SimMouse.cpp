#include "SimMouse.h"
#include <gazebo/msgs/msgs.hh>
#include <common/WallFollower.h>

SimMouse *SimMouse::instance = nullptr;
const double SimMouse::ANALOG_MAX_DIST = 0.15; // meters
const double SimMouse::MAX_FORCE = 0.006; // experimental value
const gazebo::common::Color SimMouse::grey_color{0.8, 0.8, 0.8, 1};
const RobotConfig SimMouse::CONFIG = {
        1.35255, // FRONT_ANALOG_ANGLE
        1.35255,  // BACK_ANALOG_ANGLE
        0.04,    // FRONT_SIDE_ANALOG_X
        0.024,    // FRONT_SIDE_ANALOG_Y
        -0.024,  // BACK_SIDE_ANALOG_X
        0.024,  // BACK_SIDE_ANALOG_Y
        0.05,   // FRONT_ANALOG_X
        0.12,    // MAX_SPEED
        0.02,    // MIN_SPEED
        0.15,    // WALL_THRESHOLD
};

double SimMouse::abstractForceToNewtons(double x) {
  // abstract force is from -255 to 255 per motor
  return x * MAX_FORCE / 255.0;
}

SimMouse::SimMouse() : ignore_sensor_pose_estimate(false) {}

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

  sr.walls[static_cast<int>(dir)] = range_data.front_analog < SimMouse::CONFIG.WALL_THRESHOLD;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data.front_left_analog < SimMouse::CONFIG.WALL_THRESHOLD;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data.front_right_analog < SimMouse::CONFIG.WALL_THRESHOLD;
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

Pose SimMouse::getPose() {
  return estimated_pose;
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
  std::unique_lock<std::mutex> lk(dataMutex);
  true_pose.x = msg->true_x_meters();
  true_pose.y = msg->true_y_meters();
  true_pose.yaw = msg->true_yaw_rad();

  this->left_wheel_velocity_mps = msg->left_wheel_velocity_mps();
  this->right_wheel_velocity_mps = msg->right_wheel_velocity_mps();

  this->left_wheel_angle_rad = msg->left_wheel_angle_radians();
  this->right_wheel_angle_rad = msg->right_wheel_angle_radians();

  this->range_data.front_left_analog = msg->front_left_analog();
  this->range_data.front_right_analog = msg->front_right_analog();
  this->range_data.back_left_analog = msg->back_left_analog();
  this->range_data.back_right_analog = msg->back_right_analog();
  this->range_data.front_analog = msg->front_analog();

  dataCond.notify_all();
}

bool p = false;

void SimMouse::run(double dt_s) {
  // handle updating of odometry and PID
  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s,
                                                                                 this->left_wheel_angle_rad,
                                                                                 this->right_wheel_angle_rad,
                                                                                 this->left_wheel_velocity_mps,
                                                                                 this->right_wheel_velocity_mps);

  // update row/col information. Must come first
  row = computed_row = (int) (estimated_pose.y / AbstractMaze::UNIT_DIST);
  col = computed_col = (int) (estimated_pose.x / AbstractMaze::UNIT_DIST);
  row_offset_to_edge = fmod(estimated_pose.y, AbstractMaze::UNIT_DIST);
  col_offset_to_edge = fmod(estimated_pose.x, AbstractMaze::UNIT_DIST);

  // given odometry estimate, improve estimate using sensors, then update odometry estimate to match our best estimate
  Pose kc_pose = kinematic_controller.getPose();
  estimated_pose = kc_pose;
  double est_yaw, offset;
  std::tie(est_yaw, offset) = WallFollower::estimate_pose(CONFIG, range_data, this);

  if (!ignore_sensor_pose_estimate) {
    if (p) {
//      print("allowing estimating pose from rangefinders\n");
      p = false;
    }

    estimated_pose.yaw = est_yaw;
    kinematic_controller.reset_yaw_to(estimated_pose.yaw);

    double d_wall_front;
    bool wall_in_front = false;
    if (range_data.front_analog < 0.06) {
      double yaw_error = WallFollower::yawDiff(estimated_pose.yaw, dir_to_yaw(dir));
      d_wall_front = cos(yaw_error) * range_data.front_analog + CONFIG.FRONT_ANALOG_X;
      wall_in_front = true;
    }

    switch (dir) {
      case Direction::N:
        estimated_pose.x = (col * AbstractMaze::UNIT_DIST) + offset;
        kinematic_controller.reset_x_to(estimated_pose.x);
        if (wall_in_front) {
          estimated_pose.y = (row * AbstractMaze::UNIT_DIST) + d_wall_front + AbstractMaze::HALF_WALL_THICKNESS;
          kinematic_controller.reset_y_to(estimated_pose.y);
        }
        break;
      case Direction::S:
        estimated_pose.x = (col + 1) * AbstractMaze::UNIT_DIST - offset;
        kinematic_controller.reset_x_to(estimated_pose.x);
        if (wall_in_front) {
          estimated_pose.y = ((row + 1) * AbstractMaze::UNIT_DIST) - d_wall_front - AbstractMaze::HALF_WALL_THICKNESS;
          kinematic_controller.reset_y_to(estimated_pose.y);
        }
        break;
      case Direction::E:
        estimated_pose.y = (row * AbstractMaze::UNIT_DIST) + offset;
        kinematic_controller.reset_y_to(estimated_pose.y);
        if (wall_in_front) {
          estimated_pose.x = ((col + 1) * AbstractMaze::UNIT_DIST) - d_wall_front - AbstractMaze::HALF_WALL_THICKNESS;
          kinematic_controller.reset_x_to(estimated_pose.x);
        }
        break;
      case Direction::W:
        estimated_pose.y = (row + 1) * AbstractMaze::UNIT_DIST - offset;
        kinematic_controller.reset_y_to(estimated_pose.y);
        if (wall_in_front) {
          estimated_pose.x = (col * AbstractMaze::UNIT_DIST) + d_wall_front + AbstractMaze::HALF_WALL_THICKNESS;
          kinematic_controller.reset_x_to(estimated_pose.x);
        }
        break;
    }
  } else {
    if (!p) {
//      print("Ignoring rangefinder pose estimate.\n");
      p = true;
    }
  }

  // publish status information
  gzmaze::msgs::MazeLocation maze_loc_msg;
  maze_loc_msg.set_row(getComputedRow());
  maze_loc_msg.set_col(getComputedCol());
  maze_loc_msg.set_row_offset(row_offset_to_edge);
  maze_loc_msg.set_col_offset(col_offset_to_edge);

  maze_loc_msg.set_estimated_x_meters(estimated_pose.x);
  maze_loc_msg.set_estimated_y_meters(estimated_pose.y);
  maze_loc_msg.set_estimated_yaw_rad(estimated_pose.yaw);
  std::string dir_str(1, dir_to_char(dir));
  maze_loc_msg.set_dir(dir_str);

  std::string buff;
  buff.resize(AbstractMaze::BUFF_SIZE);
  maze_mouse_string(&buff[0]);
  maze_loc_msg.set_mouse_maze_string(buff);

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

  update_markers();

}

void SimMouse::update_markers() {
  {
    ignition::msgs::Marker estimated_pose_marker;
    estimated_pose_marker.set_ns("estimated_pose");
    estimated_pose_marker.set_id(1); // constant ID makes each new marker replace the previous one
    estimated_pose_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    estimated_pose_marker.set_type(ignition::msgs::Marker::BOX);
    estimated_pose_marker.set_layer(3);
    auto material = estimated_pose_marker.mutable_material();
    ignition::msgs::Color *red = new ignition::msgs::Color();
    red->set_a(1);
    red->set_r(1);
    red->set_g(0);
    red->set_b(0);
    material->set_allocated_diffuse(red);
    ignition::msgs::Vector3d *size = estimated_pose_marker.mutable_scale();
    size->set_x(0.02);
    size->set_y(0.002);
    size->set_z(0.002);
    double x = estimated_pose.x;
    double y = -estimated_pose.y;
    double yaw = estimated_pose.yaw;
    Set(estimated_pose_marker.mutable_pose(), ignition::math::Pose3d(x, y, 0.02, 0, 0, yaw));

    ign_node.Request("/marker", estimated_pose_marker);
  }

  {
    ignition::msgs::Marker error_marker;
    error_marker.set_ns("pose_error");
    error_marker.set_id(2); // constant ID makes each new marker replace the previous one
    error_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    error_marker.set_type(ignition::msgs::Marker::LINE_STRIP);
    error_marker.set_layer(3);
    ignition::msgs::Vector3d *true_center = error_marker.add_point();
    ignition::msgs::Vector3d *estimated_center = error_marker.add_point();
    true_center->set_x(true_pose.x);
    true_center->set_y(-true_pose.y);
    true_center->set_z(.02);
    estimated_center->set_x(estimated_pose.x);
    estimated_center->set_y(-estimated_pose.y);
    estimated_center->set_z(.02);
    ignition::msgs::Material *matMsg = error_marker.mutable_material();
    matMsg->mutable_script()->set_name("Gazebo/Black");

    ign_node.Request("/marker", error_marker);
  }
}

void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps) {
  kinematic_controller.setSpeedMps(left_wheel_velocity_setpoint_mps, right_wheel_velocity_setpoint_mps);
}

void SimMouse::simInit() {
  setSpeed(0, 0);

  // we start in the middle of the first square
  kinematic_controller.reset_x_to(0.053);
  kinematic_controller.reset_y_to(0.09);
  kinematic_controller.reset_yaw_to(0.0);
  kinematic_controller.setAcceleration(0.4, 12.2);

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

