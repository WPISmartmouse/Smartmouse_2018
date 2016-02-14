#include "RealMouse.h"
#include "WallFollow.h"

RealMouse mouse;
WallFollow solver;
bool done = false;

void setup(){
  Serial.begin(9600);
  Serial.println("SETUP");
  mouse.setup();
}

void loop(){
  if (!solver.isFinished()) {
    solver.stepOnce();
  }
  else {
    if (!done) Serial.println("DONE.");
    done = true;
  }
}

