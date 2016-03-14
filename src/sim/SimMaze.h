#pragma once
#ifdef SIM

#include <mutex>
#include <condition_variable>
#include <gazebo/msgs/msgs.hh>
#include "KnownMaze.h"
#include "SimMouse.h"

class SimMaze : public KnownMaze {
  public:

    SimMaze(SimMouse *mouse);
    virtual SensorReading sense() override;
    void senseCallback(ConstLaserScanStampedPtr &msg);
    std::condition_variable senseCond;

  private:

    bool walls[4];
    float rawDistances[6];
    std::mutex senseMutex;
};
#endif
