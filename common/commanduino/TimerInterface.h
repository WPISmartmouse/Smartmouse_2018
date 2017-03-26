#pragma once

class TimerInterface {

  /// \brief the time since the start of the program in milliseconds
  public: virtual unsigned long long programTimeMs() = 0;
};
