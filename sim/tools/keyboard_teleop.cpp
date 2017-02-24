#include <iostream>
#include <cstdio>
#include <ncurses.h>

#include "SimTimer.h"

int main(int argc, char **argv) {
  // Load gazebo
  printf("Waiting for gazebo...\n");
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

  gazebo::transport::PublisherPtr controlPub;
  controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");

  initscr();
  cbreak();
  noecho();

  float kP = 0;
  float kI = 0;
  float kD = 0;

	bool keepGoing = true;
	char key;
	while (keepGoing){
    key = getch();

    float lspeed = 0;
    float rspeed = 0;
    if (key == 'w') {
      lspeed = .01;
      rspeed = .01;
		}
    else if (key == 'a') {
      lspeed = -.01;
      rspeed = .01;
    }
    else if (key == 's') {
      lspeed = -.01;
      rspeed = -.01;
    }
    else if (key == 'd') {
      lspeed = -.01;
      rspeed = .01;
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
