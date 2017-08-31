#include <iostream>
#include <common/commands/NavTestCommand.h>
#include <ignition/transport.hh>
#include <common/Flood.h>
#include <common/util.h>

#include <sim/lib/SimMouse.h>
#include <sim/lib/SimTimer.h>
#include <simulator/msgs/robot_command.pb.h>
#include <simulator/msgs/maze_location.pb.h>

int main(int argc, char *argv[]) {
  SimTimer timer;
  Command::setTimerImplementation(&timer);
  SimMouse *mouse = SimMouse::inst();

  // Create our node for communication
  bool success = mouse->node.Subscribe("time_ms", &SimTimer::simTimeCallback, &timer);
  if (!success) {
    print("Failed to subscribe to time_ms\n");
    return EXIT_FAILURE;
  }

  success = mouse->node.Subscribe("state", &SimMouse::robotStateCallback, mouse);
  if (!success) {
    print("Failed to subscribe to state\n");
    return EXIT_FAILURE;
  }

  mouse->cmd_pub = mouse->node.Advertise<smartmouse::msgs::RobotCommand>("robot_command");
  mouse->maze_location_pub = mouse->node.Advertise<smartmouse::msgs::MazeLocation>("maze_location");

  // wait for time messages to come
  while (!timer.isTimeReady());

  mouse->simInit();
  Scheduler scheduler(new NavTestCommand(new Flood(mouse)));

  bool done = false;
  unsigned long last_t = timer.programTimeMs();
  while (true) {

    unsigned long now = timer.programTimeMs();
    double dt_s = (now - last_t) / 1000.0;

    // minimum period of main loop
    if (dt_s < 0.010) {
      continue;
    }

    mouse->run(dt_s);

    if (!done) {
      done = scheduler.run();
      if (done) {
        print("done\n.");
      }
    }

    last_t = now;
  }
}

