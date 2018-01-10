#include <common/commanduino/Command.h>
#include <common/math/math.h>
#include <sim/lib/SimMouse.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/msgs/msgs.h>
#include <sim/simulator/msgs/pid_debug.pb.h>
#include <sim/simulator/msgs/robot_command.pb.h>

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
  // this lock ensures sim state updates are atomic
  // if you read state you must wait for dataCond
  std::unique_lock<std::mutex> lk(dataMutex);
  this->state_stamp = Time(msg.stamp());
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

void SimMouse::pidConstantsCallback(const smartmouse::msgs::PIDConstants &msg) {
  kinematic_controller.setParams(msg.kp(), msg.ki(), msg.kd(), msg.kffscale(), msg.kffoffset());
}

void SimMouse::serverCallback(const smartmouse::msgs::ServerControl &msg){
  if (msg.has_static_()) {
    kinematic_controller.kinematics_enabled = !msg.static_();
  }
};

void SimMouse::speedCallback(const ignition::msgs::Vector2d &msg){
  setSpeedCps(msg.x(), msg.y());
};

void SimMouse::run() {
  std::unique_lock<std::mutex> lk(dataMutex);
  dataCond.wait(lk);

  // compute dt_s from sensor stamps
  double dt_s = (state_stamp - last_state_stamp).Double();
  last_state_stamp = state_stamp;

  // handle updating of odometry and PID
  auto forces = kinematic_controller.run(dt_s, this->left_wheel_angle_rad, this->right_wheel_angle_rad, range_data);
  abstract_left_force = forces.first;
  abstract_right_force = forces.second;

  // THIS IS SUPER IMPORTANT!
  // update row/col information
  row = kinematic_controller.row;
  col = kinematic_controller.col;

  smartmouse::msgs::RobotCommand cmd;
  cmd.mutable_left()->set_abstract_force((int) abstract_left_force);
  cmd.mutable_right()->set_abstract_force((int) abstract_right_force);
  cmd_pub.Publish(cmd);

  smartmouse::msgs::PIDDebug pid;

  auto stamp = pid.mutable_stamp();
  unsigned long t_ms = Command::getTimerImplementation()->programTimeMs();
  *stamp = smartmouse::msgs::Convert((int) t_ms);
  pid.set_left_cps_setpoint(smartmouse::kc::radToCell(kinematic_controller.left_motor.setpoint_rps));
  pid.set_left_cps_actual(smartmouse::kc::radToCell(kinematic_controller.left_motor.velocity_rps));
  pid.set_right_cps_setpoint(smartmouse::kc::radToCell(kinematic_controller.right_motor.setpoint_rps));
  pid.set_right_cps_actual(smartmouse::kc::radToCell(kinematic_controller.right_motor.velocity_rps));
  pid_debug_pub.Publish(pid);
}

void SimMouse::setSpeedCps(double left_wheel_velocity_setpoint_cps, double right_wheel_velocity_setpoint_cps) {
  kinematic_controller.setSpeedCps(left_wheel_velocity_setpoint_cps, right_wheel_velocity_setpoint_cps);
}

bool SimMouse::simInit() {
  kinematic_controller.setAccelerationCpss(20);

  // we start in the middle of the first square
  resetToStartPose();

  timer = new SimTimer();
  Command::setTimerImplementation(timer);

  bool success = node.Subscribe(TopicNames::kWorldStatistics, &SimTimer::worldStatsCallback, timer);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kWorldStatistics);
    return EXIT_FAILURE;
  }

  success = node.Subscribe(TopicNames::kRobotSimState, &SimMouse::robotSimStateCallback, this);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kRobotSimState);
    return EXIT_FAILURE;
  }

  // subscribe to server control so we can listen for the "static" checkbox
  success = node.Subscribe(TopicNames::kServerControl, &SimMouse::serverCallback, this);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kServerControl);
    return EXIT_FAILURE;
  }

  success = node.Subscribe(TopicNames::kPIDConstants, &SimMouse::pidConstantsCallback, this);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kPIDConstants);
    return EXIT_FAILURE;
  }

  success = node.Subscribe(TopicNames::kPIDSetpoints, &SimMouse::speedCallback, this);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kPIDSetpoints);
    return EXIT_FAILURE;
  }

  cmd_pub = node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);
  pid_debug_pub = node.Advertise<smartmouse::msgs::PIDDebug>(TopicNames::kPIDDebug);

  // wait for time messages to come
  while (!timer->isTimeReady());

  return EXIT_SUCCESS;
}

void SimMouse::resetToStartPose() {
  reset(); // resets row, col, and dir
  kinematic_controller.reset_col_to(0.5);
  kinematic_controller.reset_row_to(0.5);
  kinematic_controller.reset_yaw_to(0.0);
}
