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

  double kP = 0.001;
  double kI = 0.0000;
  double kD = 0.0000;
  double acc = 0.0005; // m/iteration^2

  bool keepGoing = true;
  int key;

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

      smartmouse::msgs::RobotCommand cmd;
      cmd.mutable_left()->set_target_speed(1);
      cmd.mutable_left()->set_kp(kP);
      cmd.mutable_left()->set_ki(kI);
      cmd.mutable_left()->set_kd(kD);

      cmd.mutable_right()->set_target_speed(rrps);
      cmd.mutable_right()->set_kp(kP);
      cmd.mutable_right()->set_ki(kI);
      cmd.mutable_right()->set_kd(kD);

      controlPub.Publish(cmd);
    }
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &original_settings);
}
