#include <sim/simulator/msgs/robot_command.pb.h>
#include <simulator/msgs/pid_debug.pb.h>
#include <lib/Time.h>
#include "SimMouse.h"

SimMouse *SimMouse::instance = nullptr;

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

  smartmouse::msgs::RobotCommand cmd;
  cmd.mutable_left()->set_abstract_force(abstract_left_force);
  cmd.mutable_right()->set_abstract_force(abstract_right_force);
  cmd_pub.Publish(cmd);

  smartmouse::msgs::PIDDebug pid;
  auto stamp = pid.mutable_stamp();
  *stamp = Time::GetWallTime().toIgnMsg();
  pid.set_left_mps_setpoint(smartmouse::kc::radToMeters(kinematic_controller.left_motor.setpoint_rps));
  pid.set_left_mps_actual(smartmouse::kc::radToMeters(kinematic_controller.left_motor.velocity_rps));
  pid.set_right_mps_setpoint(smartmouse::kc::radToMeters(kinematic_controller.right_motor.setpoint_rps));
  pid.set_right_mps_actual(smartmouse::kc::radToMeters(kinematic_controller.right_motor.velocity_rps));
  pid_pub.Publish(pid);
}

void SimMouse::setSpeed(double left_wheel_velocity_setpoint_mps, double right_wheel_velocity_setpoint_mps) {
  kinematic_controller.setSpeedMps(left_wheel_velocity_setpoint_mps, right_wheel_velocity_setpoint_mps);
}

void SimMouse::simInit() {
  kinematic_controller.setParams(0, 0, 0, 0, 0);
  kinematic_controller.setAccelerationMpss(10);

  // we start in the middle of the first square
  resetToStartPose();

  // print warnings about any invalid publishers
  if (!cmd_pub.Valid()) {
    std::cerr << "cmd_pub is not valid! Did you forget to Advertise?" << std::endl;
  }

  if (!pid_pub.Valid()) {
    std::cerr << "pid_pub is not valid! Did you forget to Advertise?" << std::endl;
  }
}

void SimMouse::resetToStartPose() {
  reset(); // resets row, col, and dir
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(0.0);
}
