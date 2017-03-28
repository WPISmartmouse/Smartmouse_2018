#pragma once

#ifdef SIM

#include <ignition/math.hh>
#include "CommanDuino.h"
#include "SimMouse.h"
#include "Mouse.h"

class Stop : public Command {
public:
  Stop();

  Stop(unsigned long stop_time);

  void initialize();

  void execute();

  bool isFinished();

  void end();

private:
  SimMouse *mouse;
  double initial_velocity;
  unsigned long stop_time_ms = 500;
};

#endif
