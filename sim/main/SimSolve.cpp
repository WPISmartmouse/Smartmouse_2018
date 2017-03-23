#ifdef SIM
#include "CommanDuino.h"

#include "SimMouse.h"
#include "SimTimer.h"
#include "SolveCommand.h"
#include "Flood.h"
#include "WallFollow.h"
#include <iostream>
#include <gazebo/msgs/visual.pb.h>
#include <gazebo/transport/TransportTypes.hh>

int main(int argc, char* argv[]){
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected){
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
    node->Subscribe("~/mouse/pose", &SimMouse::poseCallback, SimMouse::inst());

  SimMouse::inst()->controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  SimMouse::inst()->indicatorPub = node->Advertise<gazebo::msgs::Visual>("~/visual",
      AbstractMaze::MAZE_SIZE * AbstractMaze::MAZE_SIZE);

  usleep(10000);

  SimMouse::inst()->simInit();
  Scheduler scheduler(new SolveCommand(new Flood(SimMouse::inst())));

  while (true){
    scheduler.run();
  }
}

#endif
