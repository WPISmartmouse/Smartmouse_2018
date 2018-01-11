#include <commands/WaitForStart.h>
#include <commands/Forward.h>

class Stop : public Command {
 public:
  void initialize() {
    SimMouse::inst()->setSpeedCps(0, 0);
  }
  bool isFinished() {
    return false;
  }
};

class NavTestCommand : public CommandGroup {
 public:
  NavTestCommand() : CommandGroup("NavTestGroup") {
    addSequential(new Forward());
    addSequential(new Stop());
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

