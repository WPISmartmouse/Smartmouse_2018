#include <CommanDuino.h>
#include "ArduinoTimer.h"
#include "SolveCommand.h"
#include "WaitForStart.h"
#include "WaitThenDrive.h"
#include "RealMouse.h"
#include "WallFollow.h"
#include "Flood.h"

ArduinoTimer timer;

AbstractMaze maze;
Scheduler scheduler(new SolveCommand(new Flood(RealMouse::inst())));

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
