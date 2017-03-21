#pragma once

#include "CommanDuino.h"

class Delay : public Command {
  public:
    Delay(int timeout);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    int timeout;
};
