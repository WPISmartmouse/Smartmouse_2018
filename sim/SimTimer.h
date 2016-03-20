#pragma once

#include "CommanDuino.h"

class SimTimer : public TimerInterface {
  public:
    virtual unsigned long long programTimeMs() override;
    static void simTimeCallback(ConstWorldStatisticsPtr &msg);

  private:
    static unsigned long long simTimeMs;

};
