#ifdef SIM

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>

#include "KnownMaze.h"
#include "WallFollow.h"
#include "SimMouse.h"

int main(int argc, char* argv[]){
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected){
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }

  SimMouse mouse;
  KnownMaze *maze = new KnownMaze(&mouse);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr pose_sub =
    node->Subscribe("~/mouse/pose", &SimMouse::pose_callback, &mouse);
  gazebo::transport::SubscriberPtr sense_sub =
    node->Subscribe("~/mouse/base/laser/scan", &KnownMaze::sense_callback, maze);
  mouse.control_pub = node->Advertise<gazebo::msgs::GzString>("~/mouse/control");

  mouse.simInit();

  WallFollow solver(maze);
  solver.setup();
  while (!solver.isFinished()) {
    solver.stepOnce();
    std::cin.get();
  }

  solver.teardown();
}

#endif
