#pragma once


#include <common/commanduino/CommandGroup.h>
#include <common/Mouse.h>

class ReturnToStart : public CommandGroup {
public:
  ReturnToStart(Mouse *mouse);

  void initialize();

  bool isFinished();

private:
  char *pathToStart;
  int index;
  Mouse *mouse;

};
