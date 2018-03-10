#include <commands/WaitForStart.h>
#include <commands/Forward.h>
#include <commands/ForwardN.h>
#include <commands/ForwardToCenter.h>
#include <commands/Turn.h>
#include <commands/Stop.h>

class NavTestCommand : public CommandGroup {
 public:
  NavTestCommand() : CommandGroup("NavTestGroup") {
//    addSequential(new ForwardN(6));
    addSequential(new Turn(Direction::S));
//    addSequential(new Stop(10000));
  }
};

int main(int argc, char *argv[]) {
  SimMouse *mouse = SimMouse::inst();
  mouse->simInit();

  Scheduler scheduler(new NavTestCommand());

  bool done = false;
  unsigned long last_t = mouse->timer->programTimeMs();
  while (!done) {
    unsigned long now = mouse->timer->programTimeMs();
    double dt_s = (now - last_t) / 1000.0;

    if (dt_s < 0.010) {
      continue;
    }

    mouse->run();
    done = scheduler.run();

    last_t = now;
  }
}

