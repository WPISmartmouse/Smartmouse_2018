#pragma once

#include <thread>
#include <ignition/transport/Node.hh>
#include <sim/simulator/msgs/physics_config.pb.h>

#include "Time.h"

class Server {

 public:
  void start();

  std::thread *thread;
  void RunLoop();

  void Step();

  void join();
 private:
  void OnWorldControl(const ignition::msgs::WorldControl &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);

  ignition::transport::Node *node_ptr_;
  ignition::transport::Node::Publisher world_stats_pub_;
  Time sim_time_;
  unsigned long iterations_ = 0UL;
  std::mutex physics_mutex_;
  bool pause_ = false;
  bool quit_ = false;
  unsigned int ns_per_iteration_ = 100000000u;
  unsigned long pause_at_steps_ = 0ul;
};
