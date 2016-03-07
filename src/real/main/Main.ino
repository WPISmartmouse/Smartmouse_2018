#ifdef EMBED

#include "CommanDuino.h"
#include "ArduinoTimer.h"
#include "commands/SolveCommand.h"
#include "RealMouse.h"
#include "RealMaze.h"
#include "WallFollow.h"

ArduinoTimer timer;

RealMouse mouse;
RealMaze maze(&mouse);
WallFollow solver(&maze);
Scheduler scheduler(new SolveCommand(&maze));

void setup(){
  Serial.begin(9600);
  Command::setTimerImplementation(&timer);
  mouse.setup();
}

void loop(){
  scheduler.run();
}
#endif
