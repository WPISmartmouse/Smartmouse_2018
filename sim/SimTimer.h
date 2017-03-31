#pragma once

#include <common/commanduino/CommanDuino.h>
#include <mutex>

class SimTimer : public TimerInterface {
public:
  SimTimer();

  bool isTimeReady();

  virtual unsigned long programTimeMs() override;

  void simTimeCallback(ConstWorldStatisticsPtr &msg);

private:
  unsigned long sim_time_ms;
  unsigned long last_sim_time_ms;
  unsigned long wall_time_ms;
  unsigned long last_wall_time_ms;
  double realtime_ratio;
  bool ready;
  std::mutex timeReadyMutex;

};
