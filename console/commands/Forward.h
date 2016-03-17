#pragma once
#ifdef CONSOLE

#include "CommanDuino.h"
#include "Mouse.h"

class Forward : public Command {
  public:
    Forward();
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    Mouse *mouse;

};
#endif
