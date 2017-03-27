#pragma once

#ifdef CONSOLE

#include "CommanDuino.h"

class ConsoleTimer : public TimerInterface {
  public:
    virtual unsigned long programTimeMs() override;
};
#endif
