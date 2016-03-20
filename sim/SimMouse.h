#pragma once

#ifdef SIM

#include <mutex>
#include <condition_variable>
#include "Mouse.h"
#include "AbstractMaze.h"

class SimMouse : public Mouse {
  public:

    static SimMouse *inst();

    virtual SensorReading checkWalls() override;

    void checkWallsCallback(ConstLaserScanStampedPtr &msg);

    void simInit();

    void setSpeed(float left, float right);

    ignition::math::Pose3d getPose();

    void suggestWalls(bool *walls);

    float *getRawDistances();

    void updateIndicator(int row, int col, gazebo::common::Color);
    void publishIndicators();
    void resetIndicators();

    gazebo::transport::PublisherPtr controlPub;
    gazebo::transport::PublisherPtr indicatorPub;
    void poseCallback(ConstPosePtr &msg);

    static const float MAX_SPEED;
    static const float MIN_SPEED;
    static const float WALL_DIST;

    static const gazebo::common::Color red_color;
    static const gazebo::common::Color green_color;
    static const gazebo::common::Color blue_color;
    static const gazebo::common::Color black_color;
    static const gazebo::common::Color grey_color;

  private:

    static SimMouse *instance;

    SimMouse();

    gazebo::msgs::Pose *createPose(int row, int col, float z);

    gazebo::msgs::Geometry *createCylinderGeometry(float r, float l);

    std::condition_variable checkWallsCond;
    std::mutex checkWallsMutex;

    std::condition_variable poseCond;
    std::mutex poseMutex;

    ignition::math::Pose3d pose;

    gazebo::transport::SubscriberPtr regen_sub;

		const float kP = 0.0005;
		const float kI = 0.000;
		const float kD = 0.000;

    static constexpr float INDICATOR_RAD = 0.05;
    static constexpr float INDICATOR_LEN = 0.001;

    bool walls[4];
    bool suggestedWalls[4];
    bool hasSuggestion;
    float rawDistances[3];

    gazebo::msgs::Visual *indicators[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];
};
#endif
