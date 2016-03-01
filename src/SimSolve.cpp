#ifdef SIM

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "KnownMaze.h"
#include "WallFollow.h"
#include <iostream>

int main(int argc, char* argv[]){
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected){
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr pose_sub =
    node->Subscribe("~/mouse/pose", &Mouse::pose_callback);
  gazebo::transport::SubscriberPtr sense_sub =
    node->Subscribe("~/mouse/base/laser/scan", &KnownMaze::sense_callback);
  Mouse::control_pub = node->Advertise<gazebo::msgs::GzString>("~/mouse/control");

  Mouse::simInit();

  KnownMaze *maze = new KnownMaze();
  WallFollow solver;
  solver.setup(maze);
  while (!solver.isFinished()) {
    solver.stepOnce();
    std::cin.get();
  }

  solver.teardown();
}

#endif
