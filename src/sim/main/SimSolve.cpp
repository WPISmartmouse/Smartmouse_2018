#ifdef SIM

#include "CommanDuino.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "SimMaze.h"
#include "SimMouse.h"
#include "WallFollow.h"
#include <iostream>

int main(int argc, char* argv[]){
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected){
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }

  SimMouse mouse;
  SimMaze *maze = new SimMaze(&mouse);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr poseSub =
    node->Subscribe("~/mouse/pose", &SimMouse::poseCallback, &mouse);
  gazebo::transport::SubscriberPtr senseSub = node->Subscribe("~/mouse/base/laser/scan", &SimMaze::senseCallback, maze);
  mouse.controlPub = node->Advertise<gazebo::msgs::GzString>("~/mouse/control");

  mouse.simInit();

  WallFollow solver(maze);
  solver.setup();
  solver.solve();
  solver.teardown();
}

#endif
