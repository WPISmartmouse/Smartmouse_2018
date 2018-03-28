#pragma once

#include <mutex>
#include <condition_variable>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>
#include <common/KinematicController/KinematicController.h>
#include <ignition/transport/Node.hh>

#include <sim/lib/Time.h>
#include <sim/simulator/msgs/robot_sim_state.pb.h>
#include <sim/simulator/msgs/pid_constants.pb.h>
#include <sim/lib/SimTimer.h>
#include <sim/simulator/msgs/server_control.pb.h>

class SimMouse : public Mouse {
public:
  static constexpr int rad_to_tick(double rad) {
    return static_cast<int>(rad / smartmouse::kc::RAD_PER_TICK);
  }

  static constexpr double tick_to_rad(int ticks) {
    return ticks * smartmouse::kc::RAD_PER_TICK;
  }

  static SimMouse *inst();

  virtual SensorReading checkWalls() override;

  GlobalPose getGlobalPose();

  LocalPose getLocalPose();

  std::pair<double, double> getWheelVelocitiesCPS();

  bool isStopped();

  void robotSimStateCallback(const smartmouse::msgs::RobotSimState &msg);

  void pidConstantsCallback(const smartmouse::msgs::PIDConstants &msg);

  void serverCallback(const smartmouse::msgs::ServerControl &msg);

  void speedCallback(const ignition::msgs::Vector2d &msg);

  void run();

  void setSpeedCps(double left, double right);

  bool simInit();

  void pauseSim();

  SimTimer *timer;
  ignition::transport::Node::Publisher cmd_pub;
  ignition::transport::Node::Publisher debug_state_pub;
  ignition::transport::Node::Publisher server_control_pub;
  ignition::transport::Node node;

  KinematicController kinematic_controller;

  void resetToStartPose();

private:

  SimMouse();

  static SimMouse *instance;

  double abstract_left_force;
  double abstract_right_force;
  double left_wheel_angle_rad;
  double right_wheel_angle_rad;

  RangeData range_data;

  std::condition_variable dataCond;
  std::mutex dataMutex;

  GlobalPose true_pose;
  Time state_stamp;
  Time last_state_stamp;
};

