#include <simulator/msgs/world_statistics.pb.h>
#include <simulator/msgs/msgs.h>
#include "SimTimer.h"

SimTimer::SimTimer() : ready(false), sim_time_ms(1) {
  timeReadyMutex.lock();
}

bool SimTimer::isTimeReady() {
  return timeReadyMutex.try_lock();
}

unsigned long SimTimer::programTimeMs() {
  return sim_time_ms;
}

void SimTimer::simTimeCallback(const smartmouse::msgs::WorldStatistics &msg) {
  sim_time_ms = smartmouse::msgs::ConvertMSec(msg.sim_time());

  if (!ready) {
    ready = true;
    timeReadyMutex.unlock();
  }
}

