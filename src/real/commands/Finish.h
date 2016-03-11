#pragma once
#ifdef EMBED

#include "CommanDuino.h"

class Finish : public Command {
  public:
    Finish();
    void initialize();
    bool isFinished();
};
#endif
