#pragma once

#include "CommanDuino.h"

class SimTimer : public TimerInterface {
  public:
    virtual unsigned long programTimeMs() override;
    static void simTimeCallback(ConstWorldStatisticsPtr &msg);

  private:
    static unsigned long simTimeMs;

};
