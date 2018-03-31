#include <RealMouse.h>
#include <common/commanduino/Command.h>
#include <common/commanduino/Scheduler.h>
#include <real/ArduinoTimer.h>
#include <common/core/Mouse.h>

ArduinoTimer timer;
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t_us, last_blink_us;
bool done = false;
bool on = true;

void setup() {
  delay(1000);
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  last_t_us = timer.programTimeUs();
  last_blink_us = timer.programTimeUs();

  Serial.println("setup");
}

void loop() {
  unsigned long now_us = timer.programTimeUs();
  double dt_us = now_us - last_t_us;

  if (now_us - last_blink_us > 100000) {
    last_blink_us = now_us;
    digitalWrite(RealMouse::SYS_LED, static_cast<uint8_t>(on));
    on = !on;
  }

  // minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  mouse->run(dt_us / 1e6);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();

  bool pressed = !static_cast<bool>(digitalRead(RealMouse::BUTTON_PIN));
  digitalWrite(RealMouse::LED_1, static_cast<uint8_t>(pressed));
  digitalWrite(RealMouse::LED_2, static_cast<uint8_t>(pressed));
  digitalWrite(RealMouse::LED_3, static_cast<uint8_t>(pressed));
  digitalWrite(RealMouse::LED_4, static_cast<uint8_t>(pressed));
  digitalWrite(RealMouse::LED_5, static_cast<uint8_t>(pressed));
  digitalWrite(RealMouse::LED_6, static_cast<uint8_t>(pressed));
  digitalWrite(RealMouse::LED_7, static_cast<uint8_t>(pressed));

//  print("%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f, %0.3f\r\n",
//        mouse->checkVoltage(),
//        vl,
//        vr,
//        mouse->range_data.back_left,
//        mouse->range_data.front_left,
//        mouse->range_data.gerald_left,
//        mouse->range_data.front,
//        mouse->range_data.gerald_right,
//        mouse->range_data.front_right,
//        mouse->range_data.back_right);

  mouse->setSpeedCps(1, 1);

  last_t_us = now_us;
}
