#ifdef CONSOLE
#include "Finish.h"
#include <stdio.h>

Finish::Finish() : Command("end"){}

void Finish::initialize() {
  printf("end.\n");
}

bool Finish::isFinished() {
  return true;
}
#endif
