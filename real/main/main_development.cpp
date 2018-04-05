#include <Arduino.h>
#include <real/ArduinoTimer.h>
#include <common/commanduino/CommanDuino.h>
#include <real/RealMouse.h>

#include <real/commands/WaitForStart.h>
#include <real/commands/Stop.h>
#include <real/commands/Turn.h>
#include <real/commands/ForwardN.h>

ArduinoTimer timer;
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t_us, last_blink_us;
bool done = false;
bool on = true;
bool paused = false;

class NavTestCommand : public CommandGroup {
 public:
  NavTestCommand() : CommandGroup("NavTestGroup") {
    addSequential(new WaitForStart());
    addSequential(new Stop(1000));
    addSequential(new ForwardN(3));
    addSequential(new Turn(Direction::S));
    addSequential(new Stop(10000));
  }
};

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();
  if(!digitalRead(RealMouse::BUTTON_PIN)){
    digitalWrite(RealMouse::LED_7, 1);
    delay(1000);
    while(digitalRead(RealMouse::BUTTON_PIN)){}
    mouse->calibrate();
    delay(1000);
    print("%d, %d, %d\r\n", mouse->back_left_model.adc_offset, mouse->front_left_model.adc_offset,
    mouse->gerald_left_model.adc_offset);
  }
  else {
    mouse->loadCalibrate();
    print("%d, %d, %d\r\n", mouse->back_left_model.adc_offset, mouse->front_left_model.adc_offset,
          mouse->gerald_left_model.adc_offset);
  }

  GlobalProgramSettings.quiet = false;
//  mouse->kinematic_controller.enable_sensor_pose_estimate = false;

  scheduler = new Scheduler(new NavTestCommand());

  last_t_us = timer.programTimeMs();
  last_blink_us = timer.programTimeMs();
}

void loop() {
  RealMouse::checkVoltage();

  unsigned long now_us = timer.programTimeUs();
  double dt_us = now_us - last_t_us;

  if (now_us - last_blink_us > 50000) {
    last_blink_us = now_us;
    digitalWrite(RealMouse::SYS_LED, static_cast<uint8_t>(on));
    on = !on;
  }

  // minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  auto dt_s = dt_us / 1e6;
  mouse->run(dt_s);

  // mouse->Teleop();
  if (not done) {
    done = scheduler->run();
  } else {
    mouse->setSpeedCps(0, 0);
    digitalWrite(RealMouse::SYS_LED, 1);
    digitalWrite(RealMouse::LED_2, 1);
    digitalWrite(RealMouse::LED_4, 1);
    digitalWrite(RealMouse::LED_6, 1);
  }

  static unsigned long idx = 0;
  ++idx;
//  if (idx % 10 == 0) {
//    print("%d\r\n",
//          mouse->back_left_model.adc_offset);
//  }

  last_t_us = now_us;
}
