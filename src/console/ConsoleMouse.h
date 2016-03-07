#pragma once
#ifdef CONSOLE

#include "Mouse.h"

class ConsoleMouse : public Mouse {
  public:
    virtual int forward() override;
    virtual void turnToFace(Direction d) override;
    using Mouse::turnToFace;
};
#endif
