#pragma once

#include <common/commanduino/Command.h>

class End : public Command {
  public:
    End();
    bool isFinished();
    void end();
};
