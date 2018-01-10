#pragma once

#include <common/commanduino/CommanDuino.h>
#include <mutex>
#include <sim/simulator/msgs/world_statistics.pb.h>

class SimTimer : public TimerInterface {
public:
  SimTimer();

  bool isTimeReady();

  virtual unsigned long programTimeMs() override;

  void worldStatsCallback(const smartmouse::msgs::WorldStatistics &msg);

private:
  bool ready;
  unsigned long sim_time_ms;
  std::mutex timeReadyMutex;

};
