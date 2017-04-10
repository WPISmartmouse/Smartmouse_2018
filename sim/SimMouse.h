#pragma once

#include <mutex>
#include <condition_variable>
#include <gazebo/msgs/MessageTypes.hh>
#include "msgs/msgs.h"
#include <gazebo/common/Color.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/KinematicMotorController/KinematicController.h>
#include <common/RobotConfig.h>

class SimMouse : public Mouse {
public:
  static double abstractForceToNewtons(double x);

  static const double ANALOG_MAX_DIST; // meters
  static const double MAX_FORCE;  // 16kg/cm from datasheet
  static const gazebo::common::Color grey_color;

  static const RobotConfig CONFIG;

  static SimMouse *inst();

  virtual SensorReading checkWalls() override;

  double getColOffsetToEdge();

  int getComputedRow();

  int getComputedCol();

  Pose getPose();

  Pose getExactPose();

  RangeData getRangeData();

  double getRowOffsetToEdge();

  std::pair<double, double> getWheelVelocities();

  void indicatePath(int starting_row, int starting_col,
                    std::string path, gazebo::common::Color);

  bool isStopped();

  void publishIndicators();

  void resetIndicators(gazebo::common::Color color);

  void robotStateCallback(ConstRobotStatePtr &msg);

  void run(double dt_s);

  void setSpeed(double left, double right);

  void simInit();

  void updateIndicator(int row, int col, gazebo::common::Color);

  gazebo::transport::PublisherPtr joint_cmd_pub;
  gazebo::transport::PublisherPtr indicator_pub;
  gazebo::transport::PublisherPtr maze_location_pub;

  KinematicMotorController kinematic_controller;

  bool ignore_sensor_pose_estimate;

private:

  SimMouse();

  static constexpr double INDICATOR_RAD = 0.05;
  static constexpr double INDICATOR_LEN = 0.001;
  static constexpr double INDICATOR_Z = 0.008;

  static SimMouse *instance;

  double abstract_left_force;
  double abstract_right_force;
  double left_wheel_velocity_mps;
  double right_wheel_velocity_mps;
  double left_wheel_angle_rad;
  double right_wheel_angle_rad;
  double row_offset_to_edge;
  double col_offset_to_edge;
  int computed_row;
  int computed_col;

  RangeData range_data;

  std::condition_variable dataCond;
  std::mutex dataMutex;

  Pose true_pose;
  Pose estimated_pose;

  gazebo::transport::SubscriberPtr regen_sub;

  gazebo::msgs::Visual *indicators[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];
};

