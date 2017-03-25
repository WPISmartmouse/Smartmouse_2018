#pragma once
#include "CommanDuino.h"

class Stop : public Command {
  public:
    Stop();
    bool isFinished();
    void end();
};
