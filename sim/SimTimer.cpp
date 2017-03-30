#ifdef SIM

#include <gazebo/msgs/msgs.hh>

#include "SimTimer.h"

SimTimer::SimTimer() {
  sim_time_ms = 1;
  last_wall_time_ms = (unsigned long) std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
  last_sim_time_ms = 0;
}

unsigned long SimTimer::programTimeMs() {
  // attempts to interpolate
  unsigned long current_wall_time_ms = (unsigned long) std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
  unsigned long dWallTime = current_wall_time_ms - last_wall_time_ms;
  return sim_time_ms + (unsigned long) (realtime_ratio * dWallTime);
}

void SimTimer::simTimeCallback(ConstWorldStatisticsPtr &msg) {
  sim_time_ms = (unsigned long) ((msg->sim_time().sec() * 1000) + (msg->sim_time().nsec() / 1000000));
  wall_time_ms = (unsigned long) std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();

  realtime_ratio = (sim_time_ms - last_sim_time_ms) / (double)(wall_time_ms - last_wall_time_ms);

  last_wall_time_ms = wall_time_ms;
  last_sim_time_ms = sim_time_ms;
}

#endif
