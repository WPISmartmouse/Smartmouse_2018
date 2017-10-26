#include <iostream>
#include <ignition/transport.hh>
#include <simulator/lib/common/TopicNames.h>
#include <simulator/msgs/pid_debug.pb.h>
#include <lib/Time.h>

int main() {
  ignition::transport::Node node;

  auto pub = node.Advertise<smartmouse::msgs::PIDDebug>(TopicNames::kPID);

  usleep(1000000); // 1 s

  double t = 0;
  for (size_t i = 0; i < 10000; i++) {
    smartmouse::msgs::PIDDebug pid;
    double r1 = (float)rand() / RAND_MAX;
    double r2 = (float)rand() / RAND_MAX;
    double r3 = (float)rand() / RAND_MAX;
    double r4 = (float)rand() / RAND_MAX;
    auto stamp = pid.mutable_stamp();
    int t_sec = (int) t;
    int t_nsec = (int) (t * 1e9);
    stamp->set_sec(t_sec);
    stamp->set_nsec(t_nsec);
    pid.set_left_mps_setpoint(r1);
    pid.set_left_mps_actual(r2);
    pid.set_right_mps_setpoint(r3);
    pid.set_right_mps_actual(r4);

    pub.Publish(pid);
    usleep(100000); // 100 ms
    t += 0.1;
  }
}
