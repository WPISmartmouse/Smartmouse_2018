#pragma once

#include <common/commanduino/CommanDuino.h>

class End : public Command {
public:
  End();

  bool isFinished();

  void end();
};
