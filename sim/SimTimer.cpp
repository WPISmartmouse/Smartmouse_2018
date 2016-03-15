#ifdef SIM
#include "SimTimer.h"

unsigned long long SimTimer::simTimeMs;

unsigned long long SimTimer::programTimeMs() {
  return simTimeMs;
}

void SimTimer::simTimeCallback(ConstWorldStatisticsPtr &msg) {
  simTimeMs = 0;
}
#endif
