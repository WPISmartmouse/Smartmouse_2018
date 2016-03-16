#pragma once

#ifdef SIM

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <mutex>
#include <condition_variable>
#include "Mouse.h"
#include "AbstractMaze.h"

class SimMouse : public Mouse {
  public:

    SimMouse(AbstractMaze *maze);

    virtual SensorReading sense() override;

    void senseCallback(ConstLaserScanStampedPtr &msg);

    void simInit();

    void setSpeed(float left, float right);

    ignition::math::Pose3d getPose();

    gazebo::transport::PublisherPtr controlPub;
    void poseCallback(ConstPosePtr &msg);

    static const float MAX_SPEED;
    static const float MIN_SPEED;
    static const float WALL_DIST;

  private:

    std::condition_variable senseCond;
    std::mutex senseMutex;

    std::condition_variable poseCond;
    std::mutex poseMutex;

    ignition::math::Pose3d pose;

		const float kP = 0.0005;
		const float kI = 0.000;
		const float kD = 0.000;

    bool walls[3];
    float rawDistances[5];


};
#endif
