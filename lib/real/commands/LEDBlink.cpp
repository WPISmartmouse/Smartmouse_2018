#include "LEDBlink.h"

LEDBlink::LEDBlink(const int led_pin, unsigned long blink_time) :  Command("blink"), led_pin(led_pin), blink_time(blink_time) {}

void LEDBlink::initialize(){
  digitalWrite(led_pin,1);
  blink_end = millis()+blink_time;
}

bool LEDBlink::isFinished(){
  return (millis() - blink_end > 0);
}

void LEDBlink::end(){
  digitalWrite(led_pin,0);
}
