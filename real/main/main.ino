#include <common/Flood.h>
#include <common/commands/SolveCommand.h>
#include "real/ArduinoTimer.h"
#include "real/RealMouse.h"

ArduinoTimer timer;

AbstractMaze maze;
Scheduler scheduler(new SolveCommand(new Flood(RealMouse::inst())));

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(115200);
  Serial1.begin(115200);
  Command::setTimerImplementation(&timer);
  RealMouse::inst()->setup();
}

void loop() {
  RealMouse::inst()->run();
  scheduler.run();
}
