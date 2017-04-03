#include <Arduino.h>
#include <common/commanduino/CommanDuino.h>
#include <real/ArduinoTimer.h>
#include <common/AbstractMaze.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>
#include <real/RealMouse.h>
#include <real/commands/LEDBlink.h>

ArduinoTimer timer;

AbstractMaze maze;
//Scheduler scheduler(new SolveCommand(new Flood(RealMouse::inst())));
Scheduler scheduler(new LEDBlink(RealMouse::LED_1, 100));
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
