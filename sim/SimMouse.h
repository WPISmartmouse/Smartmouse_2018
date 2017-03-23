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

class SimMouse : public Mouse {
public:
  constexpr static double ANALOG_ANGLE = 1.35255; // radians
  constexpr static double BINARY_ANGLE = 0.65; // radians
  constexpr static double LEFT_BINARY_THRESHOLD = 0.18; // meters
  constexpr static double RIGHT_BINARY_THRESHOLD = 0.18; // meters
  constexpr static double FRONT_BINARY_THRESHOLD = 0.18; // meters

  typedef struct {
    float left_analog;
    float right_analog;
    bool left_binary;
    bool right_binary;
    bool front_binary;
  } RangeData;

  gazebo::transport::PublisherPtr controlPub;
  gazebo::transport::PublisherPtr indicatorPub;

  void poseCallback(ConstRobotStatePtr &msg);

  static const float MAX_SPEED;
  static const float MIN_SPEED;
  static const float WALL_DIST;

  static const gazebo::common::Color red_color;
  static const gazebo::common::Color green_color;
  static const gazebo::common::Color blue_color;
  static const gazebo::common::Color black_color;
  static const gazebo::common::Color grey_color;

  static SimMouse *inst();

  virtual SensorReading checkWalls() override;

  void simInit();

  void setSpeed(double left, double right);

  ignition::math::Pose3d getPose();

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

  std::condition_variable checkWallsCond;
  std::mutex checkWallsMutex;

  std::condition_variable poseCond;
  std::mutex poseMutex;

  ignition::math::Pose3d pose;

  gazebo::transport::SubscriberPtr regen_sub;

  const float kP = 0.01;
  const float kI = 0.000;
  const float kD = 0.000;

  static constexpr float INDICATOR_RAD = 0.05;
  static constexpr float INDICATOR_LEN = 0.001;

  bool walls[4];
  bool suggestedWalls[4];
  bool hasSuggestion;
  RangeData range_data;

  gazebo::msgs::Visual *indicators[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];
};

#endif
