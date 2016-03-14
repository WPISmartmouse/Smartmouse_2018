#pragma once

#ifdef SIM

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <mutex>
#include <condition_variable>
#include "Mouse.h"

class SimMouse : public Mouse {
  public:

    void simInit();

    void setSpeed(float left, float right);

    gazebo::transport::PublisherPtr controlPub;
    void poseCallback(ConstPosePtr &msg);

    std::mutex poseMutex;
    std::condition_variable poseCond;
    ignition::math::Pose3d pose;

		const float kP = 0.001;
		const float kI = 0.000;
		const float kD = 0.000;

    static constexpr float MAX_SPEED = 12;
    static constexpr float MIN_SPEED = 1;

};
#endif
