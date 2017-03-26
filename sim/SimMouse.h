#pragma once

#ifdef SIM

#include <mutex>
#include <condition_variable>
#include <gazebo/msgs/MessageTypes.hh>
#include "msgs/msgs.h"
#include <gazebo/common/Color.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math.hh>
#include "Mouse.h"
#include "AbstractMaze.h"
#include "Pose.h"

class SimMouse : public Mouse {
public:
  constexpr static double ANALOG_ANGLE = 1.35255; // radians
  constexpr static double BINARY_ANGLE = 0.65; // radians
  constexpr static double LEFT_BINARY_THRESHOLD = 0.18; // meters
  constexpr static double RIGHT_BINARY_THRESHOLD = 0.18; // meters
  constexpr static double FRONT_BINARY_THRESHOLD = 0.18; // meters
  constexpr static double ANALOG_MAX_DIST = 0.15; // meters

  typedef struct {
    double left_analog;
    double right_analog;
    bool left_binary;
    bool right_binary;
    bool front_binary;
  } RangeData;

  gazebo::transport::PublisherPtr controlPub;
  gazebo::transport::PublisherPtr indicatorPub;

  void poseCallback(ConstRobotStatePtr &msg);

  static constexpr double MAX_SPEED = 0.09; // m/sec
  static constexpr double MIN_SPEED = 0.01; // m/sec
  static constexpr double ACCELERATION = 0.005; // m/iteration^2
  static constexpr double WALL_DIST = 0.115;
  static constexpr double WHEEL_RAD = 0.015;
  static constexpr double WHEEL_CIRC = 2 * WHEEL_RAD * M_PI;

  static const gazebo::common::Color red_color;
  static const gazebo::common::Color green_color;
  static const gazebo::common::Color blue_color;
  static const gazebo::common::Color black_color;
  static const gazebo::common::Color grey_color;

  double left_wheel_velocity;
  double right_wheel_velocity;

  static SimMouse *inst();

  virtual SensorReading checkWalls() override;

  void simInit();

  void setSpeed(double left, double right);

  ignition::math::Pose3d getExactPose();

  Pose getEstimatedPose();

  std::pair<double, double> getWheelVelocities();

  void suggestWalls(bool *walls);

  RangeData getRangeData();

  void indicatePath(int starting_row, int starting_col,
                    std::string path, gazebo::common::Color);

  void updateIndicator(int row, int col, gazebo::common::Color);

  void publishIndicators();

  void resetAllIndicators();

  void resetIndicators(gazebo::common::Color color);

private:

  static SimMouse *instance;

  SimMouse();

  std::condition_variable dataCond;
  std::mutex dataMutex;

  ignition::math::Pose3d pose;

  gazebo::transport::SubscriberPtr regen_sub;

  const double kP = 0.001;
  const double kI = 0.000;
  const double kD = 0.000;

  static constexpr double INDICATOR_RAD = 0.05;
  static constexpr double INDICATOR_LEN = 0.001;
  static constexpr double INDICATOR_Z = 0.002;

  bool walls[4];
  bool suggestedWalls[4];
  bool hasSuggestion;
  RangeData range_data;

  gazebo::msgs::Visual *indicators[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];
};

#endif
