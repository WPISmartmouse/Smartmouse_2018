#include "SimMouse.h"
#include <gazebo/msgs/msgs.hh>
#include <common/Mouse.h>

SimMouse *SimMouse::instance = nullptr;
const std::string SimMouse::grey_color = "Gazebo/Grey";
const std::string SimMouse::red_color = "Gazebo/Red";
const std::string SimMouse::green_color = "Gazebo/Green";
const std::string SimMouse::blue_color = "Gazebo/Blue";

double SimMouse::abstractForceToNewtons(double x) {
  // abstract force is from -255 to 255 per motor
  return x * config.MAX_FORCE / 255.0;
}

SimMouse::SimMouse() : kinematic_controller(this), range_data({}) {}

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

  sr.walls[static_cast<int>(dir)] = range_data.front < config.FRONT_WALL_THRESHOLD;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data.front_left < config.SIDE_WALL_THRESHOLD;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data.front_right < config.SIDE_WALL_THRESHOLD;
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

double SimMouse::getColOffsetToEdge() {
  return kinematic_controller.col_offset_to_edge;
}

Pose SimMouse::getPose() {
  return kinematic_controller.getPose();
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
  return kinematic_controller.row_offset_to_edge;
}

std::pair<double, double> SimMouse::getWheelVelocities() {
  return kinematic_controller.getWheelVelocities();
};

void SimMouse::indicatePath(unsigned int row, unsigned int col, std::string path, std::string color) {
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
    indicators[row][col]->mutable_material()->mutable_script()->set_name(color);
  }
}

bool SimMouse::isStopped() {
  return kinematic_controller.isStopped() && fabs(abstract_left_force) <= 5 &&
         fabs(abstract_right_force) <= RegulatedMotor::MIN_ABSTRACT_FORCE;
}

void SimMouse::publishIndicators() {
  for (unsigned int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (unsigned int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      ign_node.Request("/marker", *indicators[i][j]);
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

  this->range_data.front_left = msg->front_left_analog();
  this->range_data.front_right = msg->front_right_analog();
  this->range_data.back_left = msg->back_left_analog();
  this->range_data.back_right = msg->back_right_analog();
  this->range_data.front = msg->front_analog();

  dataCond.notify_all();
}

void SimMouse::run(double dt_s) {
  // handle updating of odometry and PID
  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s,
                                                                                 this->left_wheel_angle_rad,
                                                                                 this->right_wheel_angle_rad,
                                                                                 this->left_wheel_velocity_mps,
                                                                                 this->right_wheel_velocity_mps,
                                                                                 range_data);

  // THIS IS SUPER IMPORTANT!
  // update row/col information
  row = kinematic_controller.row;
  col = kinematic_controller.col;

  // publish status information
  gzmaze::msgs::MazeLocation maze_loc_msg;
  maze_loc_msg.set_row(row);
  maze_loc_msg.set_col(col);
  maze_loc_msg.set_row_offset(kinematic_controller.row_offset_to_edge);
  maze_loc_msg.set_col_offset(kinematic_controller.col_offset_to_edge);

  maze_loc_msg.set_estimated_x_meters(getPose().x);
  maze_loc_msg.set_estimated_y_meters(getPose().y);
  maze_loc_msg.set_estimated_yaw_rad(getPose().yaw);
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
//  static unsigned int last_row = 0, last_col = 0;
//
//  if (last_col != col || last_row != row) {
//    for (unsigned int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
//      for (unsigned int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
//        indicators[i][j]->mutable_material()->mutable_script()->set_name(grey_color);
//      }
//    }
//    indicatePath(row, col, maze->path_to_next_goal, red_color);
//    indicatePath(0, 0, maze->fastest_theoretical_route, green_color);
//    if (maze->solved) {
//      indicatePath(0, 0, maze->fastest_route, blue_color);
//    }
//
//    publishIndicators();
//
//    last_row = row;
//    last_col = col;
//  }

  {
    ignition::msgs::Marker estimated_pose_marker;
    estimated_pose_marker.set_ns("estimated_pose");
    estimated_pose_marker.set_id(1); // constant ID makes each new marker replace the previous one
    estimated_pose_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    estimated_pose_marker.set_type(ignition::msgs::Marker::BOX);
    estimated_pose_marker.set_layer(3);
    estimated_pose_marker.mutable_material()->mutable_script()->set_name(red_color);
    ignition::msgs::Vector3d *size = estimated_pose_marker.mutable_scale();
    size->set_x(0.02);
    size->set_y(0.002);
    size->set_z(0.002);
    double x = getPose().x;
    double y = -getPose().y;
    double yaw = getPose().yaw;
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
    estimated_center->set_x(getPose().x);
    estimated_center->set_y(-getPose().y);
    estimated_center->set_z(.02);
    error_marker.mutable_material()->mutable_script()->set_name("Gazebo/Black");
    ign_node.Request("/marker", error_marker);
  }
}

void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps) {
  kinematic_controller.setSpeedMps(left_wheel_velocity_setpoint_mps, right_wheel_velocity_setpoint_mps);
}

void SimMouse::simInit() {
  setSpeed(0, 0);

  kinematic_controller.setAcceleration(0.4, 12.2);

  // we start in the middle of the first square
  resetToStartPose();

  for (unsigned int i = 0; i < AbstractMaze::MAZE_SIZE; i++) {
    for (unsigned int j = 0; j < AbstractMaze::MAZE_SIZE; j++) {
      indicators[i][j] = new ignition::msgs::Marker();
      ignition::msgs::Marker *marker = indicators[i][j];
      marker->set_layer(2);
      marker->set_id(i * AbstractMaze::MAZE_SIZE + j); // constant ID makes each new marker replace the previous one
      marker->set_action(ignition::msgs::Marker::ADD_MODIFY);
      marker->set_type(ignition::msgs::Marker::CYLINDER);
      ignition::msgs::Material *matMsg = marker->mutable_material();
      matMsg->mutable_script()->set_name("Gazebo/Grey");
      ignition::msgs::Vector3d *size = marker->mutable_scale();
      size->set_x(INDICATOR_RAD);
      size->set_y(INDICATOR_RAD);
      size->set_z(INDICATOR_LEN);
      double y = -(i * AbstractMaze::UNIT_DIST + AbstractMaze::HALF_UNIT_DIST);
      double x = j * AbstractMaze::UNIT_DIST + AbstractMaze::HALF_UNIT_DIST;
      Set(marker->mutable_pose(), ignition::math::Pose3d(x, y, INDICATOR_Z, 0, 0, 0));
    }
  }

//  publishIndicators();
}

void SimMouse::resetToStartPose() {
  reset(); // resets row, col, and dir
  kinematic_controller.reset_x_to(0.053);
  kinematic_controller.reset_y_to(0.09);
  kinematic_controller.reset_yaw_to(0.0);
}
