#pragma once
#include "CommanDuino.h"

class Stop : public Command {
  public:
    Stop();
    Stop(unsigned long stop_time);
    bool isFinished();
    void end();
};
