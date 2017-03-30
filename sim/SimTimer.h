#pragma once

#include <common/commanduino/CommanDuino.h>

class SimTimer : public TimerInterface {
public:
  SimTimer();

  virtual unsigned long programTimeMs() override;

  void simTimeCallback(ConstWorldStatisticsPtr &msg);

private:
  unsigned long sim_time_ms;
  unsigned long last_sim_time_ms;
  unsigned long wall_time_ms;
  unsigned long last_wall_time_ms;
  double realtime_ratio;

};
