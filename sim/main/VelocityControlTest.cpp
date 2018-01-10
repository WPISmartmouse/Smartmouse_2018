#include <iostream>
#include <ignition/transport.hh>

#include <sim/lib/SimMouse.h>
#include <sim/lib/SimTimer.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/msgs/robot_command.pb.h>
#include <simulator/msgs/pid_debug.pb.h>
#include <simulator/msgs/server_control.pb.h>

SimMouse *mouse;

void serverCallback(const smartmouse::msgs::ServerControl &msg){
  if (msg.has_static_()) {
    mouse->kinematic_controller.kinematics_enabled = !msg.static_();
  }
};

void speedCallback(const ignition::msgs::Vector2d &msg){
  mouse->setSpeedCps(msg.x(), msg.y());
};

int main(int argc, const char **argv) {
  SimTimer timer;
  Command::setTimerImplementation(&timer);
  mouse = SimMouse::inst();

  // Create our node for communication
  bool success = mouse->node.Subscribe(TopicNames::kWorldStatistics, &SimTimer::worldStatsCallback, &timer);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kWorldStatistics);
    return EXIT_FAILURE;
  }

  success = mouse->node.Subscribe(TopicNames::kRobotSimState, &SimMouse::robotSimStateCallback, mouse);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kRobotSimState);
    return EXIT_FAILURE;
  }

  // subscribe to server control so we can listen for the "static" checkbox
  success = mouse->node.Subscribe(TopicNames::kServerControl, &serverCallback);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kServerControl);
    return EXIT_FAILURE;
  }

  success = mouse->node.Subscribe(TopicNames::kPIDConstants, &SimMouse::pidConstantsCallback, mouse);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kPIDConstants);
    return EXIT_FAILURE;
  }

  ignition::transport::Node speed_sub_node;
  success = speed_sub_node.Subscribe(TopicNames::kPIDSetpoints, &speedCallback);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kPIDSetpoints);
    return EXIT_FAILURE;
  }

  mouse->cmd_pub = mouse->node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);
  mouse->pid_debug_pub = mouse->node.Advertise<smartmouse::msgs::PIDDebug>(TopicNames::kPIDDebug);

  // wait for time messages to come
  while (!timer.isTimeReady());

  mouse->simInit();

  unsigned long last_t_ms;
  bool done = false;
  while (!done) {
    unsigned long now_ms = timer.programTimeMs();
    double dt_s = (now_ms - last_t_ms) / 1000.0;

    if (dt_s < 0.01) {
      continue;
    }

    mouse->run(dt_s);
    last_t_ms = now_ms;
  }
}

