#pragma once

#include "CommanDuino.h"
#include "RealMouse.h"

class LEDBlink : public Command {
  public:

    const static int BLUE = RealMouse::LEDB;
    const static int RED = RealMouse::LEDR;
    const static int GREEN = RealMouse::LEDG;
    const static int WHITE = RealMouse::LEDGO;

    LEDBlink(const int led_pin, unsigned long blink_time);
    void initialize();
    void execute();
    bool isFinished();
    void end();

  private:
    const int led_pin;
    unsigned long blink_time;
    unsigned long blink_end;
    RealMouse *mouse;
};
