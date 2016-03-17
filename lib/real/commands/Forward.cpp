#include "Forward.h"
#include "RealMouse.h"

Forward::Forward() : mouse(RealMouse::inst()) {}

void Forward::initialize(){}
void Forward::execute(){}
bool Forward::isFinished(){return true;}
void Forward::end(){}

