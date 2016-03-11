#pragma once
#ifdef SIM
#include "CommanDuino.h"

class Finish : public Command {
  public:
    Finish();
    void initialize();
    bool isFinished();
};
#endif
