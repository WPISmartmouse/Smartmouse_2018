#pragma once

#include <thread>
#include <ignition/transport/Node.hh>

#include <sim/simulator/lib/Time.h>
#include <sim/simulator/msgs/server_control.pb.h>
#include <sim/simulator/msgs/physics_config.pb.h>
#include <msgs/maze.pb.h>
#include <common/AbstractMaze.h>

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
  void OnMaze(const smartmouse::msgs::Maze &msg);

  ignition::transport::Node *node_ptr_;
  ignition::transport::Node::Publisher world_stats_pub_;
  Time sim_time_;
  unsigned long steps_ = 0UL;
  std::mutex physics_mutex_;
  bool pause_ = true;
  bool quit_ = false;
  unsigned int ns_of_sim_per_step_ = 1000000u;
  unsigned long pause_at_steps_ = 0ul;
  double real_time_factor_ = 1.0;
  smartmouse::msgs::Maze maze_;
};
