#pragma once

#include <thread>
#include <ignition/transport/Node.hh>

#include <msgs/server_control.pb.h>
#include <sim/simulator/lib/Time.h>
#include <sim/simulator/msgs/physics_config.pb.h>

class Server {

 public:
  void start();

  std::thread *thread_;
  void RunLoop();

  void Step();

  void join();
 private:
  void OnWorldControl(const smartmouse::msgs::ServerControl &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);

  ignition::transport::Node *node_ptr_;
  ignition::transport::Node::Publisher world_stats_pub_;
  Time sim_time_;
  unsigned long steps_ = 0UL;
  std::mutex physics_mutex_;
  bool pause_ = false;
  bool quit_ = false;
  unsigned int ns_per_step_ = 0u;
  unsigned long pause_at_steps_ = 0ul;
};
