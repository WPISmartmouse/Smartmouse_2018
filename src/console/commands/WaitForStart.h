#pragma once
#ifdef CONSOLE

#include "CommanDuino.h"

class WaitForStart : public Command {
  public:
    WaitForStart();
    void initialize();
    void execute();
    bool isFinished();
    void end();
};
#endif
