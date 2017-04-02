#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Quaternion.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Console.hh>
#include <SimMouse.h>
#include <SimTimer.h>
#include <joystick/joystick.hh>
#include <ignition/transport/Node.hh>

int main(int argc, char **argv) {
  // Load gazebo
  printf("Waiting for gazebo...\r\n");
  bool connected = gazebo::client::setup(argc, argv);
  if (!connected) {
    printf("failed to connect to gazebo. Is it running?\n");
    exit(0);
  }
  printf("Ready.\n");

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
}
