#ifdef SIM

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "KnownMaze.h"
#include "WallFollow.h"
#include <iostream>

int main(int argc, char* argv[]){
  // Load gazebo
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr pose_sub =
    node->Subscribe("~/mouse/pose", &Mouse::pose_callback);
  gazebo::transport::SubscriberPtr sense_sub =
    node->Subscribe("~/mouse/sense", &KnownMaze::sense_callback);
  Mouse::control_pub = node->Advertise<gazebo::msgs::GzString>("~/mouse/control");

  KnownMaze maze;
  WallFollow solver;
  solver.setup(maze);
  while (!solver.isFinished()) {
    solver.stepOnce();
    std::cin.get();
  }

  solver.teardown();
}

#endif
