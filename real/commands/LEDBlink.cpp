#include "LEDBlink.h"

LEDBlink::LEDBlink(const uint8_t led_pin, unsigned long blink_time) : Command("blink"), led_pin(led_pin), off(false),
                                                                  blink_time(blink_time) {}

void LEDBlink::initialize() {
  digitalWrite(led_pin, 1);
  setTimeout(2 * blink_time);
}

void LEDBlink::execute() {
  if (!off && getTime() >= blink_time) {
    digitalWrite(led_pin, 0);
    off = true;
  }
}

bool LEDBlink::isFinished() {
  return isTimedOut();
}

void LEDBlink::end() {
}
