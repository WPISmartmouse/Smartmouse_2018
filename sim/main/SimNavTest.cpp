#include <iostream>
#include <common/commands/NavTestCommand.h>
#include <ignition/transport.hh>
#include <common/core/Flood.h>
#include <common/core/util.h>

#include <sim/lib/SimMouse.h>
#include <sim/lib/SimTimer.h>
#include <simulator/msgs/robot_command.pb.h>
#include <simulator/msgs/maze_location.pb.h>
#include <simulator/lib/common/TopicNames.h>

int main(int argc, char *argv[]) {
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

  // wait for time messages to come
  while (!timer.isTimeReady());

  mouse->simInit();
  Scheduler scheduler(new NavTestCommand());

  bool done = false;
  unsigned long last_t = timer.programTimeMs();
  while (!done) {

    unsigned long now = timer.programTimeMs();
    double dt_s = (now - last_t) / 1000.0;

    // minimum period of main loop
    if (dt_s < 0.010) {
      continue;
    }

    mouse->run(dt_s);

    done = scheduler.run();

    last_t = now;
  }
}

