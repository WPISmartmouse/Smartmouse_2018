#include <ignition/math/Quaternion.hh>
#include <sim/lib/SimMouse.h>
#include <sim/lib/SimTimer.h>
#include <joystick/joystick.hh>
#include <simulator/msgs/robot_command.pb.h>
#include <simulator/lib/TopicNames.h>

int main(int argc, char **argv) {
  SimTimer timer;
  Command::setTimerImplementation(&timer);

  // Create our node for communication
  ignition::transport::Node node;
  bool success = node.Subscribe("time_ms", &SimTimer::simTimeCallback, &timer);
  if (!success) {
    printf("Failed to subscribe to time_ms\n");
    return EXIT_FAILURE;
  }

  ignition::transport::Node::Publisher controlPub;
  controlPub = node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);

  double kP = 0.001;
  double kI = 0.0000;
  double kD = 0.0000;
  double acc = 0.0005; // m/iteration^2

  bool keepGoing = true;

  double lspeed_setpoint = 0; // m/sec^2
  double rspeed_setpoint = 0; // m/sec^2
  double lspeed = 0; // m/s
  double rspeed = 0; // m/s
  const double u = .09; // m/s

  Joystick joystick("/dev/input/js0");

  if (!joystick.isFound()) {
    printf("No joystick found\n");
    return EXIT_FAILURE;
  }

  while (keepGoing) {

    usleep(1000);

    JoystickEvent event;
    if (!joystick.sample(&event)) {
      continue;
    }

    if (event.type == JS_EVENT_BUTTON && event.number == 0) {
      keepGoing = false;
    }
    if (event.type == JS_EVENT_BUTTON && event.number == 1) {
    } else if (event.type == JS_EVENT_AXIS) {
      if (event.number == 1) {
        lspeed_setpoint = -u * event.value / 30000.0;
      } else if (event.number == 4) {
        rspeed_setpoint = -u * event.value / 30000.0;
      }
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
      cmd.mutable_left()->set_target_speed(lrps);
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
}
