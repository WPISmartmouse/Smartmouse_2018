#pragma once

#include <common/commanduino/CommanDuino.h>
#include <ignition/msgs.hh>
#include <mutex>

class SimTimer : public TimerInterface {
public:
  SimTimer();

  bool isTimeReady();

  virtual unsigned long programTimeMs() override;

  void simTimeCallback(const ignition::msgs::UInt64 &msg);

private:
  unsigned long sim_time_ms;
  bool ready;
  std::mutex timeReadyMutex;

};
