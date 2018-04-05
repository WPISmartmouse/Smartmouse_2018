#include <Arduino.h>
#include <real/ArduinoTimer.h>
#include <common/commanduino/CommanDuino.h>
#include <real/RealMouse.h>

#include <real/commands/WaitForStart.h>
#include <real/commands/Stop.h>
#include <real/commands/End.h>
#include <real/commands/Forward.h>
#include <real/commands/ForwardN.h>
#include <EEPROM.h>

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
    addSequential(new Stop(10000));
  }
};

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
//  EEPROM.write(0,0);//Used to reset calibration
  mouse->setup();
  mouse->calibrate();
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

//  static unsigned long idx = 0;
//  ++idx;
//  if (idx % 10 == 0) {
//    print("%7.3f, %7.3f, %7.3f, %7.3f\r\n",
//          mouse->kinematic_controller.left_motor.setpoint_rps,
//          mouse->kinematic_controller.left_motor.velocity_rps,
//          mouse->kinematic_controller.right_motor.setpoint_rps,
//          mouse->kinematic_controller.right_motor.velocity_rps);
//  }

  last_t_us = now_us;
}
