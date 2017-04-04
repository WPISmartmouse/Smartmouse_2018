#include <gazebo/msgs/msgs.hh>

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

void SimTimer::simTimeCallback(const ignition::msgs::UInt64 &msg) {
  sim_time_ms = msg.data();

  if (!ready) {
    ready = true;
    timeReadyMutex.unlock();
  }
}

