#include <CommanDuino.h>
#include "ArduinoTimer.h"
#include "commands/SolveCommand.h"
#include "commands/WaitForStart.h"
#include "commands/WaitThenDrive.h"
#include "RealMouse.h"
#include "WallFollow.h"

ArduinoTimer timer;

AbstractMaze maze;
WallFollow solver(RealMouse::inst());
Scheduler scheduler(new WaitThenDrive());

void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(115200);
  Serial1.begin(115200);
  Command::setTimerImplementation(&timer);
  RealMouse::inst()->setup();
}

void loop(){
  RealMouse::inst()->run();
  scheduler.run();
}
