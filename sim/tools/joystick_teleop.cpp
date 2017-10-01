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

  bool keepGoing = true;

  double lforce_setpoint = 0;
  double rforce_setpoint = 0;
  double lforce = 0;
  double rforce = 0;
  const double u = 255;

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
        lforce_setpoint = -u * (float)event.value / (1 << 15);
      } else if (event.number == 4) {
        rforce_setpoint = -u * (float)event.value / (1 << 15);
      }
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
}
