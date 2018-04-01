#pragma once

#include <common/commanduino/CommanDuino.h>
#include <sim/lib/SimMouse.h>

class WaitForStart : public Command {
public:
  WaitForStart();

  void initialize();

  void execute();

  bool isFinished();

  void end();

  private:
    SimMouse *mouse;
};
