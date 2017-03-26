#pragma once

#include <gazebo/sensors/RaySensor.hh>

namespace gazebo {

  class BinaryIr : public SensorPlugin {

  public:
    BinaryIr(double threshold);

    virtual ~BinaryIr();

  private:
    double threshold;
  };
}
