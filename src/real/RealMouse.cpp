#ifdef EMBED
#include "RealMouse.h"

void RealMouse::setup(){
  pinMode(RealMouse::FORWARD_PIN, OUTPUT);
  pinMode(RealMouse::TURN_PIN, OUTPUT);

  pinMode(RealMouse::N_WALL_PIN, INPUT_PULLUP);
  pinMode(RealMouse::S_WALL_PIN, INPUT_PULLUP);
  pinMode(RealMouse::E_WALL_PIN, INPUT_PULLUP);
  pinMode(RealMouse::W_WALL_PIN, INPUT_PULLUP);
}

#endif
