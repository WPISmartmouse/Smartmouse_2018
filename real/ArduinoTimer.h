#pragma once

#include <common/commanduino/CommanDuino.h>

class ArduinoTimer : public TimerInterface {
public:
  virtual unsigned long programTimeMs() override;

  unsigned long programTimeUs();
};
