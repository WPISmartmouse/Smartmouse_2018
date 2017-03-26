#pragma once

#include <common/commanduino/Command.h>

class Calibrate : public Command {
public:
  Calibrate();

  bool isFinished();
};
