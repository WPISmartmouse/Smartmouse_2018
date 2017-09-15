#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/core/Mouse.h>
#include <common/core/Pose.h>

class ForwardN : public Command {
public:
  ForwardN(unsigned int n);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  unsigned int n;
  Mouse *mouse;
};

