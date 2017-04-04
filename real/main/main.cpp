#include <Arduino.h>
#include <common/commanduino/CommanDuino.h>
#include <real/ArduinoTimer.h>
#include <common/AbstractMaze.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>
#include <real/RealMouse.h>
#include <common/commands/RepeatCommand.h>
#include <real/commands/LEDBlink.h>

ArduinoTimer timer;
AbstractMaze maze;
//Scheduler scheduler(new SolveCommand(new Flood(RealMouse::inst())));
Scheduler scheduler(new RepeatCommand<LEDBlink, int, int>(10, RealMouse::SYS_LED, 100));
RealMouse *mouse;

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();
}

void loop() {
  mouse->run();
  scheduler.run();
}
