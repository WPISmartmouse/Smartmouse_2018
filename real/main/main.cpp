#include <Arduino.h>
#include <common/commanduino/CommanDuino.h>
#include <real/ArduinoTimer.h>
#include <common/AbstractMaze.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>
#include <real/RealMouse.h>
#include <common/commands/RepeatCommand.h>
#include <real/commands/LEDBlink.h>
#include "Forward.h"
#include "Turn.h"
#include <common/util.h>

ArduinoTimer timer;
AbstractMaze maze;
//Scheduler scheduler(new SolveCommand(new Flood(RealMouse::inst())));
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t;
bool done;

class Hack : public CommandGroup {
public:
  Hack() {
    addSequential(new Forward());
    addSequential(new Turn(Direction::S));
  }
};

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  scheduler = new Scheduler(new RepeatCommand<Forward>(3));

  last_t = timer.programTimeMs();

  print("setup\r\n");
}

void loop() {
  unsigned long now = timer.programTimeMs();
  double dt_s = (now - last_t) / 1000.0;

  // minimum period of main loop
  if (dt_s < 0.020) {
    return;
  }

  mouse->run(dt_s);
  if (!done) {
    done = scheduler->run();
  }
  else {
    mouse->setSpeed(0, 0);
    digitalWrite(RealMouse::SYS_LED, 1);
  }
  last_t = now;

}
