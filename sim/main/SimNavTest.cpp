#ifdef SIM

#include <iostream>
#include <unistd.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/msgs/visual.pb.h>

#include "SimTimer.h"
#include "Forward.h"

int main(int argc, char *argv[]) {
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected) {
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }

  SimTimer timer;
  Command::setTimerImplementation(&timer);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr timeSub =
          node->Subscribe("~/world_stats", &SimTimer::simTimeCallback);

  gazebo::transport::SubscriberPtr poseSub =
          node->Subscribe("~/mouse/state", &SimMouse::poseCallback, SimMouse::inst());

  SimMouse::inst()->controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  SimMouse::inst()->indicatorPub = node->Advertise<gazebo::msgs::Visual>("~/visual", AbstractMaze::MAZE_SIZE *
                                                                                     AbstractMaze::MAZE_SIZE);

  usleep(10000);

  SimMouse::inst()->simInit();
  Scheduler scheduler(new Forward());

  bool done = false;
  while (!done) {
    done = scheduler.run();
  }
}

#endif
