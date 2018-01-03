#pragma once

#include <mutex>
#include <condition_variable>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>
#include <common/KinematicController/KinematicController.h>
#include <ignition/transport/Node.hh>

#include <sim/simulator/msgs/robot_sim_state.pb.h>

class SimMouse : public Mouse {
public:

  static SimMouse *inst();

  virtual SensorReading checkWalls() override;

  GlobalPose getGlobalPose();

  LocalPose getLocalPose();

  std::pair<double, double> getWheelVelocities();

  bool isStopped();

  void robotSimStateCallback(const smartmouse::msgs::RobotSimState &msg);

  void run(double dt_s);

  void setSpeedCps(double left, double right);

  void simInit();

  ignition::transport::Node::Publisher cmd_pub;
  ignition::transport::Node::Publisher pid_pub;
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
};

