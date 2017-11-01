#include <iostream>
#include <ignition/transport.hh>

#include <sim/lib/SimMouse.h>
#include <sim/lib/SimTimer.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/msgs/robot_command.pb.h>
#include <simulator/msgs/pid_debug.pb.h>

SimMouse *mouse;

void callback(const ignition::msgs::Vector2d &msg){
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

  ignition::transport::Node pid_sub_node;
  success = pid_sub_node.Subscribe(TopicNames::kSpeed, &callback);
  if (!success) {
    print("Failed to subscribe to %s\n", TopicNames::kSpeed);
    return EXIT_FAILURE;
  }

  mouse->cmd_pub = mouse->node.Advertise<smartmouse::msgs::RobotCommand>(TopicNames::kRobotCommand);
  mouse->pid_pub = mouse->node.Advertise<smartmouse::msgs::PIDDebug>(TopicNames::kPID);

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

