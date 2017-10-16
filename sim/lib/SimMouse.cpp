#include <sim/simulator/msgs/robot_command.pb.h>
#include <sim/simulator/msgs/maze_location.pb.h>
#include "SimMouse.h"

SimMouse *SimMouse::instance = nullptr;
const std::string SimMouse::grey_color = "Gazebo/Grey";
const std::string SimMouse::red_color = "Gazebo/Red";
const std::string SimMouse::green_color = "Gazebo/Green";
const std::string SimMouse::blue_color = "Gazebo/Blue";

SimMouse::SimMouse() : kinematic_controller(this), range_data({}) {
  dir = Direction::N;
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

  sr.walls[static_cast<int>(dir)] = range_data.front < smartmouse::kc::FRONT_WALL_THRESHOLD;
  sr.walls[static_cast<int>(left_of_dir(dir))] = range_data.front_left < smartmouse::kc::SIDE_WALL_THRESHOLD;
  sr.walls[static_cast<int>(right_of_dir(dir))] = range_data.front_right < smartmouse::kc::SIDE_WALL_THRESHOLD;
  sr.walls[static_cast<int>(opposite_direction(dir))] = false;

  return sr;
}

GlobalPose SimMouse::getGlobalPose() {
  return kinematic_controller.getGlobalPose();
}
LocalPose SimMouse::getLocalPose() {
  return kinematic_controller.getLocalPose();
}

std::pair<double, double> SimMouse::getWheelVelocities() {
  return kinematic_controller.getWheelVelocities();
};

bool SimMouse::isStopped() {
  return kinematic_controller.isStopped() && fabs(abstract_left_force) <= 5 &&
      fabs(abstract_right_force) <= smartmouse::kc::MIN_ABSTRACT_FORCE;
}

void SimMouse::robotSimStateCallback(const smartmouse::msgs::RobotSimState &msg) {
  std::unique_lock<std::mutex> lk(dataMutex);
  true_pose.col = msg.p().col();
  true_pose.row = msg.p().row();
  true_pose.yaw = msg.p().theta();

  this->left_wheel_angle_rad = msg.left_wheel().theta();
  this->right_wheel_angle_rad = msg.right_wheel().theta();

  this->range_data.front_left = msg.front_left();
  this->range_data.front_right = msg.front_right();
  this->range_data.gerald_left = msg.gerald_left();
  this->range_data.gerald_right = msg.gerald_right();
  this->range_data.back_left = msg.back_left();
  this->range_data.back_right = msg.back_right();
  this->range_data.front = msg.front();

  dataCond.notify_all();
}

void SimMouse::run(double dt_s) {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);

  // handle updating of odometry and PID
  std::tie(abstract_left_force, abstract_right_force) = kinematic_controller.run(dt_s,
                                                                                 this->left_wheel_angle_rad,
                                                                                 this->right_wheel_angle_rad,
                                                                                 range_data);

  // THIS IS SUPER IMPORTANT!
  // update row/col information
  row = kinematic_controller.row;
  col = kinematic_controller.col;

  // publish status information
  smartmouse::msgs::MazeLocation maze_loc_msg;
  maze_loc_msg.set_row(row);
  maze_loc_msg.set_col(col);
  // TODO: finish unit conversion
  maze_loc_msg.set_row_offset(0);
  maze_loc_msg.set_col_offset(0);

  maze_loc_msg.set_estimated_x_meters(getGlobalPose().col * smartmouse::maze::UNIT_DIST_M);
  maze_loc_msg.set_estimated_y_meters(getGlobalPose().row * smartmouse::maze::UNIT_DIST_M);
  maze_loc_msg.set_estimated_yaw_rad(getGlobalPose().yaw);
  std::string dir_str(1, dir_to_char(dir));
  maze_loc_msg.set_dir(dir_str);

  std::string buff;
  buff.resize(smartmouse::maze::BUFF_SIZE);
  maze_mouse_string(&buff[0]);
  maze_loc_msg.set_mouse_maze_string(buff);

  if (!maze_loc_msg.IsInitialized()) {
    std::cerr << "Missing fields: [" << maze_loc_msg.InitializationErrorString() << "]" << std::endl;
  }

  maze_location_pub.Publish(maze_loc_msg);

  smartmouse::msgs::RobotCommand cmd;
  cmd.mutable_left()->set_abstract_force(abstract_left_force);
  cmd.mutable_right()->set_abstract_force(abstract_right_force);
  cmd_pub.Publish(cmd);
}

void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps) {
  kinematic_controller.setSpeedMps(left_wheel_velocity_setpoint_mps, right_wheel_velocity_setpoint_mps);
}

void SimMouse::simInit() {
  kinematic_controller.setParams(0, 0, 0, 0, 0);
  kinematic_controller.setAccelerationMpss(2.0);

  // we start in the middle of the first square
  resetToStartPose();
}

void SimMouse::resetToStartPose() {
  reset(); // resets row, col, and dir
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(0.0);
}
