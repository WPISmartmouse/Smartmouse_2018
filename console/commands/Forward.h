#pragma once

#include <common/commanduino/CommanDuino.h>
#include <common/Mouse.h>

class Forward : public Command {
public:
  Forward();

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  Mouse *mouse;

};
