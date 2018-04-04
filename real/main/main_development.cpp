#include <Arduino.h>
#include <real/ArduinoTimer.h>
#include <common/commanduino/CommanDuino.h>
#include <real/RealMouse.h>

#include <real/commands/WaitForStart.h>
#include <real/commands/Stop.h>

ArduinoTimer timer;
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t_us, last_blink;
bool done = false;
bool on = true;
bool paused = false;

class NavTestCommand : public CommandGroup {
 public:
  NavTestCommand() : CommandGroup("NavTestGroup") {
    addSequential(new WaitForStart());
    addSequential(new Stop(10000));
  }
};

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  GlobalProgramSettings.quiet = false;

  scheduler = new Scheduler(new NavTestCommand());

  last_t_us = timer.programTimeMs();
  last_blink = timer.programTimeMs();
}

void loop() {
  unsigned long now = timer.programTimeMs();
  double dt_s = (now - last_t_us) / 1000.0;

  if (not done and now - last_blink > 100) {
    last_blink = now;
    digitalWrite(RealMouse::SYS_LED, static_cast<uint8_t>(on));
    on = !on;
  }

  // minimum period of main loop
  if (dt_s < 0.010) {
    return;
  }

  mouse->run(dt_s);

  if (not done) {
    done = scheduler->run();
  } else {
    mouse->setSpeedCps(0, 0);
    digitalWrite(RealMouse::SYS_LED, 1);
    digitalWrite(RealMouse::LED_2, 1);
    digitalWrite(RealMouse::LED_4, 1);
    digitalWrite(RealMouse::LED_6, 1);
  }
  last_t_us = now;
}
