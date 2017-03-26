#pragma once

#include <common/commanduino/Command.h>

class Stop : public Command {
public:
  Stop();

  bool isFinished();

  void end();
};
