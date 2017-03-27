#ifdef SIM

#include <gazebo/msgs/msgs.hh>
#include "SimTimer.h"

unsigned long SimTimer::simTimeMs;

unsigned long SimTimer::programTimeMs() {
  return simTimeMs;
}

void SimTimer::simTimeCallback(ConstWorldStatisticsPtr &msg) {
  simTimeMs = (unsigned long) ((msg->sim_time().sec() * 1000) + (msg->sim_time().nsec() / 1000000));
}
#endif
