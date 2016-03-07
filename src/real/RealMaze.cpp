#ifdef EMBED
#include <Arduino.h>
#include "RealMaze.h"

RealMaze::RealMaze(RealMouse *mouse) : KnownMaze(mouse) {}

//this function should block until all the data about walls has been collected
SensorReading RealMaze::sense(){
  //#TODO actual sensor code here
}
#endif
