#pragma once

#include "common/commanduino/CommanDuino.h"

class ArduinoTimer : public TimerInterface {
  virtual unsigned long long programTimeMs() override;
};
