#pragma once
#include <common/commanduino/CommanDuino.h>

class Stop : public Command {
  public:
    Stop();
    Stop(unsigned long stop_time);
    bool isFinished();
    void end();
};
