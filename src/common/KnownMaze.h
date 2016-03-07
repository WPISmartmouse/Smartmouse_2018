#pragma once

#include "Mouse.h"
#include "Direction.h"
#include "SensorReading.h"
#include "AbstractMaze.h"

class KnownMaze : public AbstractMaze {

  public:

    KnownMaze(Mouse *mouse);

    virtual SensorReading sense() = 0;
    bool is_mouse_blocked();
    bool is_mouse_blocked(Direction dir);

    static constexpr float WALL_DIST = 0.125;
};
