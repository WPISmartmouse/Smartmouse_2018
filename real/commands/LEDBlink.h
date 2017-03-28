#pragma once

#include <common/commanduino/CommanDuino.h>
#include "RealMouse.h"

class LEDBlink : public Command {
public:

  const static int B = RealMouse::LEDB;
  const static int R = RealMouse::LEDR;
  const static int G = RealMouse::LEDG;
  const static int W = RealMouse::LEDGO;

  LEDBlink(const int led_pin, unsigned long blink_time);

  void initialize();

  bool isFinished();

  void end();

private:
  const int led_pin;
  unsigned long blink_time;
  unsigned long blink_end;
};
