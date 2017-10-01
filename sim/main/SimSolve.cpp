#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>

#include <sim/lib/SimTimer.h>
#include <simulator/msgs/maze_location.pb.h>
#include <simulator/msgs/robot_command.pb.h>
#include <simulator/lib/TopicNames.h>
#include <sim/lib/SimMouse.h>

int main(int argc, char *argv[]) {
  SimTimer timer;
  Command::setTimerImplementation(&timer);
  SimMouse *mouse = SimMouse::inst();

  // Create our node for communication
  bool success = mouse->node.Subscribe(TopicNames::kWorldStatistics, &SimTimer::simTimeCallback, &timer);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kWorldStatistics);
    return EXIT_FAILURE;
  }

  success = mouse->node.Subscribe(TopicNames::kRobotSimState, &SimMouse::robotSimStateCallback, mouse);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kRobotSimState);
    return EXIT_FAILURE;
  }

  mouse->cmd_pub = mouse->node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);
  mouse->maze_location_pub = mouse->node.Advertise<smartmouse::msgs::MazeLocation>(TopicNames::kMazeLocation);

  // wait for time messages to come
  while (!timer.isTimeReady());

  mouse->simInit();

  Scheduler scheduler(new SolveCommand(new Flood(mouse)));

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
//    done = scheduler.run();
    last_t = now;
  }
}

