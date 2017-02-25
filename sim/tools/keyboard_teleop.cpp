#include <iostream>
#include <cstdio>
#include <ncurses.h>

#include "SimTimer.h"
#include "msgs/msgs.h"

typedef const boost::shared_ptr<const gzmaze::msgs::RobotState> RobotStatePtr;

void stateCallback(RobotStatePtr &msg) {
  printf("%f %f\r\n", msg->left_wheel(), msg->right_wheel());
}

int main(int argc, char **argv) {
  // Load gazebo
  printf("Waiting for gazebo...\r\n");
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

  gazebo::transport::SubscriberPtr stateSub =
    node->Subscribe("~/mouse/state", &stateCallback);

  gazebo::transport::PublisherPtr controlPub;
  controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");

  initscr();
  cbreak();
  noecho();

  float kP = 0.04;
  float kI = 0.01;
  float kD = 0;

	bool keepGoing = true;
	char key;
  float lspeed = 0;
  float rspeed = 0;
	while (keepGoing){
    key = getch();

    if (key == 'w') {
      lspeed += 1;
      rspeed += 1;
		}
    else if (key == 'a') {
      lspeed += 1;
      rspeed -= 1;
    }
    else if (key == 's') {
      lspeed -= 1;
      rspeed -= 1;
    }
    else if (key == 'd') {
      lspeed -= 1;
      rspeed += 1;
    }
    else if (key == 'q') {
      lspeed = 0;
      rspeed = 0;
    }

    printf("%f, %f\r\n", lspeed, rspeed);

    gazebo::msgs::JointCmd left;
    left.set_name("mouse::left_wheel_joint");
    left.mutable_velocity()->set_target(lspeed);
    left.mutable_velocity()->set_p_gain(kP);
    left.mutable_velocity()->set_i_gain(kI);
    left.mutable_velocity()->set_d_gain(kD);
    controlPub->Publish(left);

    gazebo::msgs::JointCmd right;
    right.set_name("mouse::right_wheel_joint");
    right.mutable_velocity()->set_target(rspeed);
    right.mutable_velocity()->set_p_gain(kP);
    right.mutable_velocity()->set_i_gain(kI);
    right.mutable_velocity()->set_d_gain(kD);
    controlPub->Publish(right);
	}
}
