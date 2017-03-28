#ifdef SIM

#include <common/commanduino/CommanDuino.h>
#include <common/commands/SolveCommand.h>
#include <common/Flood.h>

#include "SimMouse.h"
#include "SimTimer.h"

int main(int argc, char* argv[]) {
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
          node->Subscribe("~/mouse/state", &SimMouse::robotStateCallback, SimMouse::inst());

  SimMouse::inst()->controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  SimMouse::inst()->indicatorPub = node->Advertise<gazebo::msgs::Visual>("~/visual", AbstractMaze::MAZE_SIZE *
                                                                                     AbstractMaze::MAZE_SIZE);

  // .01 seconds
  usleep(10000);

  SimMouse::inst()->simInit();

  Scheduler *scheduler;
  scheduler = new Scheduler(new SolveCommand(new Flood(SimMouse::inst())));

  bool done = false;
  while (!done) {
    done = scheduler->run();
  }
}

#endif
