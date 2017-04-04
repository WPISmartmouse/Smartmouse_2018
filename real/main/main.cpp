#include <Arduino.h>
#include <common/commanduino/CommanDuino.h>
#include <real/ArduinoTimer.h>
#include <common/AbstractMaze.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>
#include <real/RealMouse.h>
#include <common/commands/RepeatCommand.h>
#include <real/commands/LEDBlink.h>
#include <common/util.h>

ArduinoTimer timer;
AbstractMaze maze;
//Scheduler scheduler(new SolveCommand(new Flood(RealMouse::inst())));
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t;

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  delay(2000);

  scheduler = new Scheduler(new RepeatCommand<LEDBlink, int, int>(10, RealMouse::SYS_LED, 500));

//  scheduler = new Scheduler(new LEDBlink(RealMouse::SYS_LED, 100));
  last_t = timer.programTimeMs();

  print("setup\n");
}

void loop() {
  unsigned long now = timer.programTimeMs();
  double dt_s = (now - last_t) / 1000.0;

  // minimum period of main loop
  if (dt_s < 0.010) {
    return;
  }

  mouse->run(dt_s);
  scheduler->run();
}
