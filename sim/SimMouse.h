#pragma once

#ifdef SIM

#include <mutex>
#include <condition_variable>
#include <gazebo/msgs/MessageTypes.hh>
#include "msgs/msgs.h"
#include <gazebo/common/Color.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math.hh>
#include <common/Mouse.h>
#include <common/Pose.h>
#include <common/KinematicMotorController/KinematicController.h>

class SimMouse : public Mouse {
public:
  static double abstractForceToNewtons(double x);

  static constexpr double ANALOG_ANGLE = 1.35255; // radians
  static constexpr double BINARY_ANGLE = 0.65; // radians
  static constexpr double LEFT_BINARY_THRESHOLD = 0.18; // meters
  static constexpr double RIGHT_BINARY_THRESHOLD = 0.18; // meters
  static constexpr double FRONT_BINARY_THRESHOLD = 0.18; // meters
  static constexpr double ANALOG_MAX_DIST = 0.15; // meters
  static constexpr double SIDE_ANALOG_X = 0.04; // meters
  static constexpr double SIDE_ANALOG_Y = 0.024; // meters
  static constexpr double SIDE_BINARY_X = 0.043; // meters
  static constexpr double SIDE_BINARY_Y = 0.022; // meters
  static constexpr double FRONT_BINARY_Y = 0.045; // meters
  static constexpr double MAX_SPEED = 0.18; // m/sec
  static constexpr double MIN_SPEED = 0.005; // m/sec
  static constexpr unsigned long CONTROL_PERIOD_MS = 20;
  static constexpr double CONTROL_PERIOD_S = CONTROL_PERIOD_MS / 1000.0;
  static constexpr double START_ACCELERATION_MPSS = 0.24; // m/sec^2
  static constexpr double BRAKE_ACCELERATION_MPSS = 0.50;// m/sec^2
  static constexpr double START_ACCELERATION = START_ACCELERATION_MPSS * CONTROL_PERIOD_S; // m/it^2
  static constexpr double BRAKE_ACCELERATION = BRAKE_ACCELERATION_MPSS * CONTROL_PERIOD_S; // m/it^2
  static constexpr double WALL_DIST = 0.125;
  static constexpr double MAX_FORCE = 0.016;  // 16kg/cm from datasheet
  static const gazebo::common::Color grey_color;

  static SimMouse *inst();

  typedef struct {
    double left_analog;
    double right_analog;
    bool left_binary;
    bool right_binary;
    bool front_binary;
  } RangeData;

  virtual SensorReading checkWalls() override;

  double getColOffsetToEdge();

  int getComputedRow();

  int getComputedCol();

  Pose getEstimatedPose();

  Pose getExactPose();

  RangeData getRangeData();

  double getRowOffsetToEdge();

  std::pair<double, double> getWheelVelocities();

  void indicatePath(int starting_row, int starting_col,
                    std::string path, gazebo::common::Color);

  void publishIndicators();

  void resetIndicators(gazebo::common::Color color);

  void robotStateCallback(ConstRobotStatePtr &msg);

  void run(unsigned long time_ms);

  void setSpeed(double left, double right);

  void simInit();

  void updateIndicator(int row, int col, gazebo::common::Color);

  gazebo::transport::PublisherPtr joint_cmd_pub;
  gazebo::transport::PublisherPtr indicator_pub;
  gazebo::transport::PublisherPtr maze_location_pub;

  KinematicMotorController kinematic_controller;

private:

  SimMouse();

  static constexpr double INDICATOR_RAD = 0.05;
  static constexpr double INDICATOR_LEN = 0.001;
  static constexpr double INDICATOR_Z = 0.008;

  static SimMouse *instance;

  bool walls[4];
  bool suggestedWalls[4];
  bool hasSuggestion;
  double left_wheel_velocity;
  double right_wheel_velocity;
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

  gazebo::transport::SubscriberPtr regen_sub;

  gazebo::msgs::Visual *indicators[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];
};

#endif
