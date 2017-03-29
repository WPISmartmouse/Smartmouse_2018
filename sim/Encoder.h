#pragma once

class Encoder {

public:
  Encoder();
  void init(int pin_a, int pin_b);
  int read();

};