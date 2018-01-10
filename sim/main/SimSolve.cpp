#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include <common/core/Flood.h>

#include <sim/lib/SimTimer.h>
#include <sim/lib/SimMouse.h>

int main(int argc, char *argv[]) {
  SimMouse *mouse = SimMouse::inst();

  mouse->simInit();

  Scheduler scheduler(new SolveCommand(new Flood(mouse)));

  bool done = false;
  unsigned long last_t = mouse->timer->programTimeMs();
  while (!done) {
    unsigned long now = mouse->timer->programTimeMs();
    double dt_s = (now - last_t) / 1000.0;

    // minimum period of main loop
    if (dt_s < 0.010) {
      continue;
    }

    mouse->run();
    done = scheduler.run();
    last_t = now;
  }
}

