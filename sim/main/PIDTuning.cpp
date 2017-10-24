#include <iostream>
#include <ignition/transport.hh>

#include <sim/lib/SimMouse.h>
#include <sim/lib/SimTimer.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/msgs/robot_command.pb.h>
#include <common/argparse/argparse.hpp>
#include <simulator/msgs/pid_debug.pb.h>

int main(int argc, const char **argv) {
  SimTimer timer;
  Command::setTimerImplementation(&timer);
  SimMouse *mouse = SimMouse::inst();

  // Create our node for communication
  bool success = mouse->node.Subscribe(TopicNames::kWorldStatistics, &SimTimer::worldStatsCallback, &timer);
  if (!success) {
    print("Failed to subscribe to time_ms\n");
    return EXIT_FAILURE;
  }

  success = mouse->node.Subscribe(TopicNames::kRobotSimState, &SimMouse::robotSimStateCallback, mouse);
  if (!success) {
    print("Failed to subscribe to state\n");
    return EXIT_FAILURE;
  }

  mouse->cmd_pub = mouse->node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);
  mouse->pid_pub = mouse->node.Advertise<smartmouse::msgs::PIDDebug>(TopicNames::kPID);

  // wait for time messages to come
  while (!timer.isTimeReady());

  mouse->simInit();

  const unsigned long ms_per_setpoint = 2000;
  for (double i = 0; i < smartmouse::kc::MAX_HARDWARE_SPEED_MPS; i += 0.1) {
    mouse->setSpeed(i, smartmouse::kc::MAX_HARDWARE_SPEED_MPS - i);
    std::cout << "Setpoint: " << i << std::endl;

    unsigned long start_ms = timer.programTimeMs();
    unsigned long last_t_ms = timer.programTimeMs();
    unsigned long now_ms = start_ms;

    do {
      now_ms = timer.programTimeMs();
      double dt_s = (now_ms - last_t_ms) / 1000.0;

      if (dt_s < 0.01) {
        continue;
      }

      mouse->run(dt_s);

      last_t_ms = now_ms;
    } while (now_ms - start_ms < ms_per_setpoint);

  }
}

