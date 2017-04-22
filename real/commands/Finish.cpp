#include "Finish.h"
#include <Arduino.h>

Finish::Finish(AbstractMaze *maze) : Command("end"), maze(maze) {}

void Finish::initialize() {
  print(maze->fastest_route);
  setTimeout(2000);
  t = getTime();
  pin_id = RealMouse::LED_1;
  on = true;
  mouse->setSpeed(0, 0);
}

void Finish::execute() {
  if (getTime() - t > BLINK_TIME) {

    on = !on;
    t = getTime();

    if (on) {
      digitalWrite(pin_id, 1);
    } else {
      digitalWrite(pin_id, 0);
      pin_id++;
      if (pin_id > RealMouse::LED_8) {
        pin_id = RealMouse::LED_1;
      }
    }
}
}

bool Finish::isFinished() {
  return isTimedOut();
}
