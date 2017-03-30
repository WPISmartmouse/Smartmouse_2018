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

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr timeSub =
          node->Subscribe("~/world_stats", &SimTimer::simTimeCallback, &timer);

  gazebo::transport::SubscriberPtr poseSub =
          node->Subscribe("~/mouse/state", &SimMouse::robotStateCallback, SimMouse::inst());

  SimMouse::inst()->controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");
  SimMouse::inst()->indicatorPub = node->Advertise<gazebo::msgs::Visual>("~/visual", AbstractMaze::MAZE_SIZE *
                                                                                     AbstractMaze::MAZE_SIZE);
  gazebo::transport::PublisherPtr maze_location_pub = node->Advertise<gzmaze::msgs::MazeLocation>("~/maze_location");

  // .01 seconds
  usleep(10000);

  SimMouse *mouse = SimMouse::inst();
  mouse->simInit();
  Scheduler scheduler(new NavTestCommand());

  bool done = false;
  while (!done) {
    done = scheduler.run();
    gzmaze::msgs::MazeLocation msg;
    msg.set_row(mouse->getComputedRow());
    msg.set_col(mouse->getComputedCol());
    msg.set_row_offset(mouse->getRowOffsetToEdge());
    msg.set_col_offset(mouse->getColOffsetToEdge());

    Pose estimated_pose = mouse->getEstimatedPose();
    msg.set_estimated_x_meters(estimated_pose.x);
    msg.set_estimated_y_meters(estimated_pose.y);
    msg.set_estimated_yaw_rad(estimated_pose.yaw);
    std::string dir_str(1, dir_to_char(mouse->getDir()));
    msg.set_dir(dir_str);

    if (!msg.IsInitialized()) {
      std::cerr << "Missing fields: [" <<  msg.InitializationErrorString() << "]" << std::endl;
    }

    maze_location_pub->Publish(msg);

    mouse->run(timer.programTimeMs());
  }
}

#endif
