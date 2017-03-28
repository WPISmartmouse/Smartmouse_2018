#pragma once
#include "CommanDuino.h"

class End : public Command {
  public:
    End();
    bool isFinished();
    void end();
};
