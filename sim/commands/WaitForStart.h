#pragma once
#include <common/commanduino/CommanDuino.h>

class WaitForStart : public Command {
  public:
    WaitForStart();
    void initialize();
    void execute();
    bool isFinished();
    void end();
};
