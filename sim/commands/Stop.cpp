#ifdef SIM

#include "Stop.h"

Stop::Stop() : mouse(SimMouse::inst()) {}

void Stop::initialize(){
  setTimeout(2000);
}

void Stop::execute(){
  mouse->setSpeed(0,0);
}

bool Stop::isFinished(){
  double left_velocity, right_velocity;
  std::tie(left_velocity, right_velocity) = mouse->getWheelVelocities();
  return (left_velocity <= SimMouse::MIN_SPEED && right_velocity <= SimMouse::MIN_SPEED) && isTimedOut();
}

void Stop::end(){
}
#endif
