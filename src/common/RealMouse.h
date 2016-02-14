#ifdef EMBED
#include <Arduino.h>
#include "Mouse.h"

class RealMouse : public Mouse {
  public:
    /** @interit_doc */
    void forward();

    /** @interit_doc */
    void turn_to_face(Direction d);

    /** @interit_doc */
    void turn_to_face(char c);

    /** runs setup things like pin initializes */
    void setup();

  private:
    const static int FORWARD_PIN = 8;
    const static int TURN_PIN = 9;
    const static int N_WALL_PIN = 4;
    const static int E_WALL_PIN = 5;
    const static int S_WALL_PIN = 6;
    const static int W_WALL_PIN = 7;
};
#endif
