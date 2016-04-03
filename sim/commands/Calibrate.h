#pragma once
#include "CommanDuino.h"

class Calibrate : public Command {
  public:
    Calibrate();
    bool isFinished();
};
