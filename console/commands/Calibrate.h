#pragma once
#include <common/commanduino/CommanDuino.h>

class Calibrate : public Command {
  public:
    Calibrate();
    bool isFinished();
};
