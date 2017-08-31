#pragma once

#include <thread>
#include <ignition/transport/Node.hh>
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
  void OnPhysics(const ignition::msgs::Physics &msg);

  ignition::transport::Node *node_ptr_;
  ignition::transport::Node::Publisher world_stats_pub_;
  Time sim_time_;
  int32_t iterations_ = 0UL;
  std::mutex physics_mutex_;
  bool pause_;
  bool quit_;
  int ns_per_iteration_ = 100000000u;
  unsigned int pause_at_steps_;
};
