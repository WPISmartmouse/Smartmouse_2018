#include <RealMouse.h>
#include <common/commanduino/Command.h>
#include <common/commanduino/Scheduler.h>
#include <real/ArduinoTimer.h>

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
  RealMouse::checkVoltage();

  unsigned long now_us = timer.programTimeUs();
  double dt_us = now_us - last_t_us;

  if (now_us - last_blink_us > 100000) {
    last_blink_us = now_us;
    digitalWrite(RealMouse::SYS_LED, static_cast<uint8_t>(on));
    on = !on;
  }

  // minimum period of main loop
  if (dt_us < 2000) {
    return;
  }

  mouse->run(dt_us / 1e6);
  double vl, vr;
  std::tie(vl, vr) = mouse->getWheelVelocities();

  print("%0.3f, %0.3f\r\n", vl, vr);

  mouse->setSpeedCps(1, 0);

  last_t_us = now_us;
}
