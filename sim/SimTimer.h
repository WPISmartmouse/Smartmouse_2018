#pragma once

#include "CommanDuino.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

class SimTimer : public TimerInterface {
  public:
    virtual unsigned long long programTimeMs() override;
    static void simTimeCallback(ConstWorldStatisticsPtr &msg);

  private:
    static unsigned long long simTimeMs;

};
