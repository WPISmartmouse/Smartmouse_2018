#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/msgs/visual.pb.h>
#include <common/commands/NavTestCommand.h>
#include <ignition/transport.hh>
#include <common/core/Flood.h>
#include <common/core/util.h>

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

  ignition::transport::Node ign_node;
  bool success = ign_node.Subscribe("/time_ms", &SimTimer::simTimeCallback, &timer);
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

