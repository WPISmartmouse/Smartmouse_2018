#pragma once

#include <Arduino.h>
#include "Mouse.h"
#include "AbstractMaze.h"

class RealMouse : public Mouse {
  public:

    /** runs setup things like pin initializes */
    void setup();

    RealMouse(AbstractMaze *maze);

    virtual SensorReading sense() override;

    const static int startButtonPin = 0;

  private:
    const static int FORWARD_PIN = 8;
    const static int TURN_PIN = 9;
    const static int N_WALL_PIN = 4;
    const static int E_WALL_PIN = 5;
    const static int S_WALL_PIN = 6;
    const static int W_WALL_PIN = 7;
};
