#pragma once

#include <common/commanduino/CommanDuino.h>

class Stop : public Command {
public:
  Stop();

  Stop(unsigned long stop_time);

  void initialize();

  bool isFinished();

  void end();

private:
  unsigned long stop_time;
};
