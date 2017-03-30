#ifdef SIM

#include <iostream>
#include <unistd.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/msgs/visual.pb.h>
#include <common/commands/NavTestCommand.h>
#include <ignition/transport/Node.hh>

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

  gazebo::transport::SubscriberPtr timeSub =
          node->Subscribe("~/world_stats", &SimTimer::simTimeCallback, &timer);

  gazebo::transport::SubscriberPtr poseSub =
          node->Subscribe("~/mouse/state", &SimMouse::robotStateCallback, SimMouse::inst());

  mouse->joint_cmd_pub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  mouse->indicator_pub = node->Advertise<gazebo::msgs::Visual>("~/visual", AbstractMaze::MAZE_SIZE *
                                                                                     AbstractMaze::MAZE_SIZE);
  mouse->maze_location_pub = node->Advertise<gzmaze::msgs::MazeLocation>("~/maze_location");

  // .01 seconds
  usleep(10000);

  mouse->simInit();
  Scheduler scheduler(new NavTestCommand());

  bool done = false;
  while (!done) {
    done = scheduler.run();
    mouse->run(timer.programTimeMs());
  }
}

#endif
