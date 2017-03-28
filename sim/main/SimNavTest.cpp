#ifdef SIM

#include <iostream>
#include <unistd.h>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/msgs/visual.pb.h>
#include <SimMouse.h>
#include "maze_location.pb.h"
#include <ignition/transport/Node.hh>

#include "SimTimer.h"
#include "NavTestCommand.h"

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
          node->Subscribe("~/mouse/state", &SimMouse::robotStateCallback, SimMouse::inst());

  SimMouse::inst()->controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  SimMouse::inst()->indicatorPub = node->Advertise<gazebo::msgs::Visual>("~/visual", AbstractMaze::MAZE_SIZE *
                                                                                     AbstractMaze::MAZE_SIZE);
  gazebo::transport::PublisherPtr maze_location_pub = node->Advertise<gzmaze::msgs::MazeLocation>("~/maze_location");

  // .01 seconds
  usleep(10000);

  SimMouse::inst()->simInit();
  Scheduler scheduler(new NavTestCommand());

  bool done = false;
  while (!done) {
    done = scheduler.run();
    gzmaze::msgs::MazeLocation msg;
    msg.set_row(SimMouse::inst()->getComputedRow());
    msg.set_col(SimMouse::inst()->getComputedCol());
    msg.set_row_offset(SimMouse::inst()->getRowOffsetToEdge());
    msg.set_col_offset(SimMouse::inst()->getColOffsetToEdge());
    std::string dir_str(1, dir_to_char(SimMouse::inst()->getDir()));
    msg.set_dir(dir_str);
    maze_location_pub->Publish(msg);
  }
}

#endif
