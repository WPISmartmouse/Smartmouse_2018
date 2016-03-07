#pragma once
#ifdef EMBED

#include "RealMouse.h"
#include "KnownMaze.h"

class RealMaze : public KnownMaze {
  public:
    RealMaze(RealMouse *mouse);
    virtual SensorReading sense() override;

};
#endif
