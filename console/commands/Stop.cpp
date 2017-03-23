#ifdef SIM

#include "Stop.h"

Stop::Stop() : mouse(SimMouse::inst()) {}

void Stop::initialize(){
}

void Stop::execute(){
  mouse->setSpeed(0,0);
}

bool Stop::isFinished(){
  double left_velocity, right_velocity;
  std::tie(left_velocity, right_velocity) = mouse->getWheelVelocities();
  printf("%f, %f\n", left_velocity, right_velocity);
  return left_velocity == 0 && right_velocity == 0;
}

void Stop::end(){
}
#endif
