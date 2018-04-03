/**
 * Test the PID values by executing a fixed velocity profile
 */

#include <real/RealMouse.h>
#include <common/commanduino/Command.h>
#include <common/commanduino/Scheduler.h>
#include <real/ArduinoTimer.h>
#include <common/KinematicController/VelocityProfile.h>

ArduinoTimer timer;
Scheduler *scheduler;
RealMouse *mouse;
unsigned long last_t_us;
bool done = false;
smartmouse::kc::VelocityProfile *profile;

void setup() {
  delay(1000);
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  mouse->kinematic_controller.enable_sensor_pose_estimate = false;

  last_t_us = timer.programTimeUs();

  GlobalPose start(0, 0, 0);
  profile = new smartmouse::kc::VelocityProfile(start, {1, 0, 0});
}

void loop() {
  RealMouse::checkVoltage();

  unsigned long now_us = timer.programTimeUs();
  double dt_us = now_us - last_t_us;

  // minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  auto dt_s = dt_us / 1e6;
  mouse->run(dt_s);
  // time since start of the motion in seconds
  // since this program only does this once it's just millis converted to seconds
  double t_s = static_cast<double>(millis()) / 1000.0;
  double v = profile->compute_forward_velocity(t_s);
  print("% 5.3f, %i, % 5.3f\r\n", v, mouse->kinematic_controller.left_motor.setpoint_rps, mouse->kinematic_controller.left_motor.velocity_rps);
  mouse->setSpeedCps(v, v);

  last_t_us = now_us;
}
