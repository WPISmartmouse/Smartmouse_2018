#ifdef CONSOLE
#include "Mouse.h"

class ConsoleMouse : public Mouse {
  public:
    virtual int forward() override;
    virtual void turn_to_face(Direction d) override;
};
#endif

