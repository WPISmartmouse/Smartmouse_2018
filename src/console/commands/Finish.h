#pragma once
#ifdef CONSOLE

#include "CommanDuino.h"

class Finish : public Command {
  public:
    Finish();
    void initialize();
    bool isFinished();
};
#endif
