#include <iostream>
#include <ignition/transport.hh>
#include <simulator/lib/common/TopicNames.h>
#include <simulator/msgs/pid_debug.pb.h>
#include <lib/Time.h>

int main() {
  ignition::transport::Node node;

  auto pub = node.Advertise<smartmouse::msgs::PIDDebug>(TopicNames::kPID);


  for (size_t i = 0; i < 10; i++) {
    smartmouse::msgs::PIDDebug pid;
    double r1 = (float)rand() / RAND_MAX;
    double r2 = (float)rand() / RAND_MAX;
    double r3 = (float)rand() / RAND_MAX;
    double r4 = (float)rand() / RAND_MAX;
    pid.set_left_mps_setpoint(r1);
    pid.set_left_mps_actual(r2);
    pid.set_right_mps_setpoint(r3);
    pid.set_right_mps_actual(r4);

    usleep(1000000);
    pub.Publish(pid);
  }
}
