#pragma once

#include <common/commanduino/CommanDuino.h>

class ConsoleTimer : public TimerInterface {
public:
  virtual unsigned long programTimeMs() override;
};

