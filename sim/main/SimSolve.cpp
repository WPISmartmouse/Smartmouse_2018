#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>
#include <ignition/transport/Node.hh>
#include <common/util.h>

#include "SimMouse.h"
#include "SimTimer.h"

int main(int argc, char *argv[]) {
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected) {
    print("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }

  SimTimer timer;
  Command::setTimerImplementation(&timer);
  SimMouse *mouse = SimMouse::inst();

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  bool success = mouse->ign_node.Subscribe("/time_ms", &SimTimer::simTimeCallback, &timer);
  if (!success) {
    print("Failed to subscribe to /time_ms\n");
    return EXIT_FAILURE;
  }

  gazebo::transport::SubscriberPtr poseSub = node->Subscribe("~/mouse/state", &SimMouse::robotStateCallback, mouse);

  mouse->joint_cmd_pub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  mouse->maze_location_pub = node->Advertise<gzmaze::msgs::MazeLocation>("~/maze_location");

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
    done = scheduler.run();
    last_t = now;
  }
}

