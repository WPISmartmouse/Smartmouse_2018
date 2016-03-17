#include "Turn.h"
#include <stdio.h>
#include "RealMouse.h"

Turn::Turn(Direction dir) : mouse(RealMouse::inst()), dir(dir) {}

void Turn::initialize(){}

void Turn::execute(){}

bool Turn::isFinished(){}

void Turn::end(){}

