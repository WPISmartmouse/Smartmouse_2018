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
#include <common/IRSensorModeling/Model.h>

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
  RangeData<int> range_data_adc;

  /** store this as meters interally. the RoboSimState msg will be in ADC values **/
  RangeData<double> range_data_m;

  void resetToStartPose();

// copied from the output of compute_calibration_curves
  smartmouse::ir::ModelParams back_left_model{1.387919, 0.043049, 0.0, 0.0};
  smartmouse::ir::ModelParams front_left_model{1.290353, 0.031279, 0.0, 0.0};
  smartmouse::ir::ModelParams gerald_left_model{1.307681, 0.032274, 0.0, 0.0};
  smartmouse::ir::ModelParams front_model{1.408344, 0.044311, 0.0, 0.0};
  smartmouse::ir::ModelParams gerald_right_model{1.325542, 0.034024, 0.0, 0.0};
  smartmouse::ir::ModelParams front_right_model{1.266203, 0.027914, 0.0, 0.0};
  smartmouse::ir::ModelParams back_right_model{1.340392, 0.036548, 0.0, 0.0};

private:

  SimMouse();

  static SimMouse *instance;

  double abstract_left_force;
  double abstract_right_force;
  double left_wheel_angle_rad;
  double right_wheel_angle_rad;

  std::condition_variable dataCond;
  std::mutex dataMutex;

  GlobalPose true_pose;
  Time state_stamp;
  Time last_state_stamp;
};

