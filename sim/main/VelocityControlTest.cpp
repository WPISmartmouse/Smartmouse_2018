#include <sim/lib/SimMouse.h>

int main(int argc, const char **argv) {
  SimMouse *mouse = SimMouse::inst();
  mouse->simInit();

  unsigned long last_t_ms;
  bool done = false;
  while (!done) {
    unsigned long now_ms = mouse->timer->programTimeMs();
    double dt_s = (now_ms - last_t_ms) / 1000.0;

    if (dt_s < 0.01) {
      continue;
    }

    mouse->run();
    last_t_ms = now_ms;
  }
}

