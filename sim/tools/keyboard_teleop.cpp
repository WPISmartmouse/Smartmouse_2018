#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <ignition/transport/Node.hh>

#include <sim/lib/SimTimer.h>
#include <sim/lib/SimMouse.h>
#include <simulator/msgs/robot_command.pb.h>
#include <simulator/lib/TopicNames.h>

int main(int argc, char **argv) {

  struct termios original_settings;
  tcgetattr(STDIN_FILENO, &original_settings);

  struct termios tty;
  tcgetattr(STDIN_FILENO, &tty);
  tty.c_lflag &= ~ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &tty);

  printf("Press W,A,S,D,X then enter to send a command.\n");

  SimTimer timer;
  Command::setTimerImplementation(&timer);

  // Create our node for communication
  ignition::transport::Node node;
  ignition::transport::Node::Publisher controlPub;
  controlPub = node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);

  bool keepGoing = true;
  int key;

  double lforce_setpoint = 0;
  double rforce_setpoint = 0;
  double lforce = 0;
  double rforce = 0;
  const double u = .09;
  while (keepGoing) {
    key = std::cin.get();

    if (key == 'w') {
      lforce_setpoint += u;
      rforce_setpoint += u;
    } else if (key == 'a') {
      lforce_setpoint += u;
      rforce_setpoint -= u;
    } else if (key == 's') {
      lforce_setpoint = 0;
      rforce_setpoint = 0;
    } else if (key == 'd') {
      lforce_setpoint -= u;
      rforce_setpoint += u;
    } else if (key == 'x') {
      lforce_setpoint -= u;
      rforce_setpoint -= u;
    } else if (key == 'q') {
      keepGoing = false;
    } else {
    }

    if (rforce != rforce_setpoint or lforce != lforce_setpoint) {
      rforce = rforce_setpoint;
      lforce = lforce_setpoint;

      smartmouse::msgs::RobotCommand cmd;
      cmd.mutable_left()->set_abstract_force(lforce);
      cmd.mutable_right()->set_abstract_force(rforce);

      controlPub.Publish(cmd);
    }
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &original_settings);
}
