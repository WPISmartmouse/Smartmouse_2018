#include <Arduino.h>
#include <real/ArduinoTimer.h>
#include <common/commanduino/CommanDuino.h>
#include <real/RealMouse.h>

#include <real/commands/WaitForStart.h>
#include <real/commands/Stop.h>
#include <real/commands/ForwardToCenter.h>
#include <real/commands/TurnInPlace.h>
#include <real/commands/Turn.h>
#include <real/commands/ForwardN.h>
#include <real/commands/Forward.h>

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
    addSequential(new Forward());
//    addSequential(new ForwardN(2));
//    addSequential(new ForwardToCenter());
    addSequential(new Turn(Direction::S));
//    addSequential(new Stop(10000));
  }
};

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

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

  if (now_us - last_blink_us > 100000) {
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

  if (not paused and not done) {
    // one of these should be commented out
//    mouse->Teleop();
    done = scheduler->run();
  } else {
    mouse->setSpeedCps(0, 0);
    digitalWrite(RealMouse::SYS_LED, 1);
    digitalWrite(RealMouse::LED_2, 1);
    digitalWrite(RealMouse::LED_4, 1);
    digitalWrite(RealMouse::LED_6, 1);
  }

  if (Serial1.available()) {
    char c = static_cast<char>(Serial1.read());
    if (c == 'p') {
      paused = !paused;
    }
  }

//  auto p = mouse->getGlobalPose();
//  print_slow("%4.3f, %4.3f, %4.3f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %d, %d\r\n",
//             p.row,
//             p.col,
//             p.yaw,
//             mouse->range_data_m.back_left,
//             mouse->range_data_m.front_left,
//             mouse->range_data_m.gerald_left,
//             mouse->range_data_m.front,
//             mouse->range_data_m.gerald_right,
//             mouse->range_data_m.front_right,
//             mouse->range_data_m.back_right,
//             mouse->kinematic_controller.sense_left_wall,
//             mouse->kinematic_controller.sense_right_wall);

  last_t_us = now_us;
}
