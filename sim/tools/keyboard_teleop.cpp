#include <iostream>
#include <math.h>
#include <cstdio>
#include <termios.h>
#include <unistd.h>

#include "SimTimer.h"
#include "msgs/msgs.h"

typedef const boost::shared_ptr<const gzmaze::msgs::RobotState> RobotStatePtr;
constexpr double WHEEL_RAD = 0.015;
constexpr double WHEEL_CIRC = 2 * WHEEL_RAD * M_PI;
int filter_prints = 0;

void stateCallback(RobotStatePtr &msg) {
  // radians/sec
  double l = msg->left_wheel();
  double r = msg->right_wheel();
  double lmps = l / (2 * M_PI) * WHEEL_CIRC;
  double rmps = r / (2 * M_PI) * WHEEL_CIRC;

  double x = msg->position().position().x();
  double y = msg->position().position().y();
  ignition::math::Quaterniond q = gazebo::msgs::ConvertIgn(msg->position().orientation());
  const double yaw = q.Yaw();

	filter_prints += 1;

  if (filter_prints % 100 == 0) {
    printf("l:%f, r:%f, x:%f, y:%f, yaw:%f\r\n", lmps, rmps, x, y, yaw);
  }
}

int main(int argc, char **argv) {

	struct termios original_settings;
	tcgetattr(STDIN_FILENO, &original_settings);

	struct termios tty;
	tcgetattr(STDIN_FILENO, &tty);
	tty.c_lflag &= ~ECHO;
	tcsetattr(STDIN_FILENO, TCSANOW, &tty);

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

  double kP = 0.04;
  double kI = 0.01;
  double kD = 0;

	bool keepGoing = true;
	char key;

  // meter's per second
  double lspeed = 0;
  double rspeed = 0;
	while (keepGoing){
    key = std::cin.get();

    if (key == 'w') {
      lspeed += 0.1;
      rspeed += 0.1;
		}
    else if (key == 'a') {
      lspeed += 0.1;
      rspeed -= 0.1;
    }
    else if (key == 's') {
      lspeed = 0;
      rspeed = 0;
    }
    else if (key == 'd') {
      lspeed -= 0.1;
      rspeed += 0.1;
    }
    else if (key == 'x') {
      lspeed -= 0.1;
      rspeed -= 0.1;
    }
    else if (key == 'q') {
      keepGoing = false;
    }


    double lrps = lspeed * (2 * M_PI) / WHEEL_CIRC;
    double rrps = rspeed * (2 * M_PI) / WHEEL_CIRC;

    printf("%f, %f\r\n", lrps, rrps);

    gazebo::msgs::JointCmd left;
    left.set_name("mouse::left_wheel_joint");
    left.mutable_velocity()->set_target(lrps);
    left.mutable_velocity()->set_p_gain(kP);
    left.mutable_velocity()->set_i_gain(kI);
    left.mutable_velocity()->set_d_gain(kD);
    controlPub->Publish(left);

    gazebo::msgs::JointCmd right;
    right.set_name("mouse::right_wheel_joint");
    right.mutable_velocity()->set_target(rrps);
    right.mutable_velocity()->set_p_gain(kP);
    right.mutable_velocity()->set_i_gain(kI);
    right.mutable_velocity()->set_d_gain(kD);
    controlPub->Publish(right);
	}

	tcsetattr(STDIN_FILENO, TCSANOW, &original_settings);
}
