#ifdef SIM
#include "CommanDuino.h"

#include "SimMouse.h"
#include "SimTimer.h"
#include "SolveCommand.h"
#include "Flood.h"
#include <StepSolveCommand.h>

int main(int argc, char* argv[]) {
  // Load gazebo
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected) {
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }

  bool step = false;
  if (argc == 2) {
    if (strncmp("--step", argv[1], 6) == 0) {
      step = true;
    }
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

  // .01 seconds
  usleep(10000);

  SimMouse::inst()->simInit();

  Scheduler *scheduler;
  if (step) {
    scheduler = new Scheduler(new StepSolveCommand(new Flood(SimMouse::inst())));
  }
  else {
    scheduler = new Scheduler(new SolveCommand(new Flood(SimMouse::inst())));
  }

  bool done = false;
  while (!done) {
    done = scheduler->run();
  }
}

#endif
