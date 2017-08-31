#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Quaternion.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Console.hh>
#include <ignition/transport/Node.hh>

#include <sim/lib/SimTimer.h>
#include <sim/lib/SimMouse.h>

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
  if (!connected) {
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }
  printf("connected to gazebo. Press W,A,S,D,X then enter to send a command.\n");

  SimTimer timer;
  Command::setTimerImplementation(&timer);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  ignition::transport::Node ign_node;
  bool success = ign_node.Subscribe("/time_ms", &SimTimer::simTimeCallback, &timer);
  if (!success) {
    printf("Failed to subscribe to /time_ms\n");
    return EXIT_FAILURE;
  }

  gazebo::transport::PublisherPtr controlPub;
  controlPub = node->Advertise<gazebo::msgs::JointCmd>("~/mouse/joint_cmd");

  double kP = 0.001;
  double kI = 0.0000;
  double kD = 0.0000;
  double acc = 0.0005; // m/iteration^2

  bool keepGoing = true;
  char key;

  double lspeed_setpoint = 0; // m/sec^2
  double rspeed_setpoint = 0; // m/sec^2
  double lspeed = 0; // m/s
  double rspeed = 0; // m/s
  const double u = .09; // m/s
  while (keepGoing) {
    key = std::cin.get();

    if (key == 'w') {
      lspeed_setpoint += u;
      rspeed_setpoint += u;
    } else if (key == 'a') {
      lspeed_setpoint += u;
      rspeed_setpoint -= u;
    } else if (key == 's') {
      lspeed_setpoint = 0;
      rspeed_setpoint = 0;
    } else if (key == 'd') {
      lspeed_setpoint -= u;
      rspeed_setpoint += u;
    } else if (key == 'x') {
      lspeed_setpoint -= u;
      rspeed_setpoint -= u;
    } else if (key == 'q') {
      keepGoing = false;
    } else {
    }

    // Handle acceleration
    while (rspeed != rspeed_setpoint or lspeed != lspeed_setpoint) {

      if (rspeed < rspeed_setpoint) {
        rspeed = std::min(rspeed + acc, rspeed_setpoint);
      } else if (rspeed > rspeed_setpoint) {
        rspeed = std::max(rspeed - acc, rspeed_setpoint);
      }

      if (lspeed < lspeed_setpoint) {
        lspeed = std::min(lspeed + acc, lspeed_setpoint);
      } else if (lspeed > lspeed_setpoint) {
        lspeed = std::max(lspeed - acc, lspeed_setpoint);
      }

      double lrps = SimMouse::meterToRad(lspeed);
      double rrps = SimMouse::meterToRad(rspeed);

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
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &original_settings);
}
