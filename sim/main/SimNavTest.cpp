#ifdef SIM

#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/msgs/visual.pb.h>
#include <common/commands/NavTestCommand.h>
#include <ignition/transport.hh>

#include "SimMouse.h"
#include "SimTimer.h"

int main(int argc, char *argv[]) {
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected) {
    printf("failed to connect to gazebo. Is it running?\n");
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
    printf("Failed to subscribe to /time_ms\n");
    return EXIT_FAILURE;
  }

  gazebo::transport::SubscriberPtr poseSub = node->Subscribe("~/mouse/state", &SimMouse::robotStateCallback,
                                                             SimMouse::inst());

  mouse->joint_cmd_pub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  mouse->indicator_pub = node->Advertise<gazebo::msgs::Visual>("~/visual", AbstractMaze::MAZE_SIZE *
                                                                           AbstractMaze::MAZE_SIZE);
  mouse->maze_location_pub = node->Advertise<gzmaze::msgs::MazeLocation>("~/maze_location");

  // wait for time messages to come
  while (!timer.isTimeReady());

  mouse->simInit();
  Scheduler scheduler(new NavTestCommand());

  bool done = false;
  while (!done) {
    done = scheduler.run();
    unsigned long time_ms = timer.programTimeMs();
    mouse->run(time_ms);
  }

  printf("Commands done.");

  while (true) {
    unsigned long time_ms = timer.programTimeMs();
    mouse->run(time_ms);
  }
}

#endif
