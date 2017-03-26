#pragma once

#ifdef CONSOLE

#include "common/commanduino/TimerInterface.h"

class ConsoleTimer : public TimerInterface {
  public:
    virtual unsigned long long programTimeMs() override;
};
#endif
