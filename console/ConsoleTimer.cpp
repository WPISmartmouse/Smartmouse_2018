#ifdef CONSOLE
#include "ConsoleTimer.h"
#include <chrono>

unsigned long ConsoleTimer::programTimeMs() {
  return (unsigned  long)std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
#endif
