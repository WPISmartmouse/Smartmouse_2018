#include <real/RealMouse.h>
#include <real/ArduinoTimer.h>
#include <common/core/Mouse.h>

ArduinoTimer timer;
RealMouse *mouse;
unsigned long last_t_us;
bool done = false;
bool on = true;

void setup() {
  Command::setTimerImplementation(&timer);
  mouse = RealMouse::inst();
  mouse->setup();

  last_t_us = timer.programTimeUs();
}

void loop() {
  static unsigned long idx = 0;
  unsigned long now_us = timer.programTimeUs();
  double dt_us = now_us - last_t_us;

  // minimum period of main loop
  if (dt_us < 1500) {
    return;
  }

  ++idx;
  if (idx % 500 == 0) {
    auto p = mouse->getLocalPose();
    print("%f %f %f %f %f %f, %f\r\n",
          mouse->range_data_m.back_left,
          mouse->range_data_m.front_left,
          mouse->range_data_m.gerald_left,
          mouse->range_data_m.front,
          mouse->range_data_m.gerald_right,
          mouse->range_data_m.front_right,
          mouse->range_data_m.back_right);

    print("%6i %6i %6i %6i %6i %6i, %6i\r\n",
          mouse->range_data_adc.back_left,
          mouse->range_data_adc.front_left,
          mouse->range_data_adc.gerald_left,
          mouse->range_data_adc.front,
          mouse->range_data_adc.gerald_right,
          mouse->range_data_adc.front_right,
          mouse->range_data_adc.back_right);

    print("%d %d\r\n",
          mouse->kinematic_controller.sense_left_wall,
          mouse->kinematic_controller.sense_right_wall);

    print("%f %f %f\r\n",
          p.to_left,
          p.to_back,
          p.yaw_from_straight);
  }

  mouse->run(dt_us / 1e6);

  last_t_us = now_us;
}
