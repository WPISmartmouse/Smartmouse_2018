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

class SimMouse : public Mouse {
public:
  static double radPerSecToMetersPerSec(double x);

  static double metersPerSecToRadPerSec(double x);

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
  static constexpr double FWD_ACCELERATION = 0.002; // m/iteration^2
  static constexpr double STOP_ACCELERATION = 0.01; // m/iteration^2
  static constexpr double TRACK_WIDTH = 0.0626; // m
  static constexpr double WALL_DIST = 0.125;
  static constexpr double WHEEL_RAD = 0.015;
  static constexpr double WHEEL_CIRC = 2 * WHEEL_RAD * M_PI;
  static const gazebo::common::Color grey_color;

  static SimMouse *inst();

  typedef struct {
    double left_analog;
    double right_analog;
    bool left_binary;
    bool right_binary;
    bool front_binary;
  } RangeData;

  gazebo::transport::PublisherPtr controlPub;
  gazebo::transport::PublisherPtr indicatorPub;

  void robotStateCallback(ConstRobotStatePtr &msg);

  virtual SensorReading checkWalls() override;

  void simInit();

  void setSpeed(double left, double right);

  ignition::math::Pose3d getExactPose();

  Pose getEstimatedPose();

  double getRowOffsetToEdge();

  double getColOffsetToEdge();

  int getComputedRow();

  int getComputedCol();

  std::pair<double, double> getWheelVelocities();

  void suggestWalls(bool *walls);

  RangeData getRangeData();

  void indicatePath(int starting_row, int starting_col,
                    std::string path, gazebo::common::Color);

  void updateIndicator(int row, int col, gazebo::common::Color);

  void publishIndicators();

  void resetIndicators(gazebo::common::Color color);

private:

  SimMouse();

  static constexpr double INDICATOR_RAD = 0.05;
  static constexpr double INDICATOR_LEN = 0.001;
  static constexpr double INDICATOR_Z = 0.008;

  const double kP = 0.002;
  const double kI = 0.000;
  const double kD = 0.000;

  static SimMouse *instance;

  std::condition_variable dataCond;
  std::mutex dataMutex;

  ignition::math::Pose3d pose;

  gazebo::transport::SubscriberPtr regen_sub;

  bool walls[4];
  bool suggestedWalls[4];
  bool hasSuggestion;
  RangeData range_data;

  double left_wheel_velocity;
  double right_wheel_velocity;
  double left_wheel_angle;
  double right_wheel_angle;
  double row_offset_to_edge;
  double col_offset_to_edge;
  int computed_row;
  int computed_col;

  gazebo::msgs::Visual *indicators[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];
};

#endif
