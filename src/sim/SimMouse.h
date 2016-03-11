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

    gazebo::transport::PublisherPtr controlPub;
    void poseCallback(ConstPosePtr &msg);

    std::mutex poseMutex;
    std::condition_variable poseCond;
    ignition::math::Pose3d pose;

};
#endif
