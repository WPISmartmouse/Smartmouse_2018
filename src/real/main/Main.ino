#ifdef EMBED
#include <CommanDuino.h>
#include "commands/SolveCommand.h"
#include "RealMouse.h"
#include "RealMaze.h"
#include "WallFollow.h"

Scheduler scheduler(new SolveCommand());

ArduinoTimer timer;
Command::setTimerImplementation(&timer);

RealMouse mouse;
RealMaze maze(&mouse);
WallFollow solver(&maze);

void setup(){
  Serial.begin(9600);
  mouse.setup();

}

void loop(){
  scheduler.run();
}
#endif
