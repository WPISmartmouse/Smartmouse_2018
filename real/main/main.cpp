#include <Arduino.h>
#include <common/commanduino/CommanDuino.h>
#include <real/ArduinoTimer.h>
#include <real/RealMouse.h>
#include <common/core/util.h>
#include <common/core/Flood.h>
#include <common/commands/SolveCommand.h>

ArduinoTimer timer;
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t_us, last_blink_us;
bool done = false;
bool on = true;
bool paused = false;

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  GlobalProgramSettings.quiet = false;

  scheduler = new Scheduler(new SolveCommand(new Flood(mouse)));

  last_t_us = timer.programTimeMs();
  last_blink_us = timer.programTimeMs();
}

void loop() {
  RealMouse::checkVoltage();

  unsigned long now_us = timer.programTimeUs();
  double dt_us = (now_us - last_t_us) / 1000.0;

  if (now_us - last_blink_us > 10000) {
    last_blink_us = now_us;
    digitalWrite(RealMouse::SYS_LED, static_cast<uint8_t>(on));
    on = !on;
  }

  // minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  mouse->run(dt_us / 1e6);

  if (!done) {
#ifdef PROFILE
    unsigned long t0 = micros();
#endif
    done = scheduler->run();
#ifdef PROFILE
    Serial.print("Schedule, ");
    Serial.println(micros() - t0);
#endif
  } else {
    mouse->setSpeedCps(0, 0);
    digitalWrite(RealMouse::SYS_LED, 1);
  }
  last_t_us = now_us;
}
