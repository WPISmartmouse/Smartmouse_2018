#ifdef EMBED

#include "CommanDuino.h"
#include "ArduinoTimer.h"
#include "commands/SolveCommand.h"
#include "RealMouse.h"
#include "WallFollow.h"

ArduinoTimer timer;

AbstractMaze maze;
RealMouse mouse(&maze);
WallFollow solver(&mouse);
Scheduler scheduler(new SolveCommand(new WallFollow(&mouse)));

void setup(){
  Serial.begin(9600);
  Command::setTimerImplementation(&timer);
  mouse.setup();
}

void loop(){
  scheduler.run();
}
#endif
