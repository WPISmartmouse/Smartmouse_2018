#ifdef SIM

#include "Stop.h"

Stop::Stop() : mouse(SimMouse::inst()), stop_time_ms(0) {}

Stop::Stop(unsigned long stop_time) : mouse(SimMouse::inst()), stop_time_ms(stop_time) {}

void Stop::initialize() {
  double initial_left_velocity, initial_right_velocity;
  std::tie(initial_left_velocity, initial_right_velocity) = mouse->getWheelVelocities();
  initial_velocity = (initial_left_velocity + initial_right_velocity) / 2;
  setTimeout(stop_time_ms);
}

void Stop::execute() {
  if (getTime() < stop_time_ms) {
    double kT = ((stop_time_ms - getTime()) / (float) stop_time_ms) * initial_velocity;
    mouse->setSpeed(kT, kT);
  } else {
    mouse->setSpeed(0, 0);
  }
}

bool Stop::isFinished() {
  double left_velocity, right_velocity;
  std::tie(left_velocity, right_velocity) = mouse->getWheelVelocities();
  return (left_velocity <= 0 && right_velocity <= 0) && isTimedOut();
}

void Stop::end() {
}

#endif
