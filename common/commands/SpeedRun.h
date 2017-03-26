#pragma once

#include <common/commanduino/CommandGroup.h>
#include <common/Mouse.h>

class SpeedRun: public CommandGroup {
  public:
    SpeedRun(Mouse *mouse);
    void initialize();
    bool isFinished();

  private:
    Mouse *mouse;
    char *path;
    int index;
};
