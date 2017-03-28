#pragma once

#include "CommanDuino.h"

class ArduinoTimer : public TimerInterface {
  virtual unsigned long long programTimeMs() override;
};
