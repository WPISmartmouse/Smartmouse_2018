#ifdef SIM
#include "SimTimer.h"

unsigned long long SimTimer::simTimeMs;

unsigned long long SimTimer::programTimeMs() {
  return simTimeMs;
}

void SimTimer::simTimeCallback(ConstWorldStatisticsPtr &msg) {
  simTimeMs = (msg->sim_time().sec()/1000.0) + (msg->sim_time().nsec() * 1000);
}
#endif
