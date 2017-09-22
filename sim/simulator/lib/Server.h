#pragma once

#include <thread>

#include <ignition/transport/Node.hh>

#include <common/AbstractMaze.h>
#include <sim/simulator/lib/Time.h>
#include <sim/simulator/msgs/server_control.pb.h>
#include <sim/simulator/msgs/physics_config.pb.h>
#include <sim/simulator/msgs/internal_physics_state.pb.h>
#include <sim/simulator/msgs/maze.pb.h>
#include <sim/simulator/msgs/robot_sim_state.pb.h>
#include <sim/simulator/msgs/robot_command.pb.h>
#include <msgs/robot_description.pb.h>

class Server {

 public:
  void start();
  void RunLoop();
  void Step();
  void join();

  std::thread *thread_;
 private:
  void OnServerControl(const smartmouse::msgs::ServerControl &msg);
  void OnPhysics(const smartmouse::msgs::PhysicsConfig &msg);
  void OnMaze(const smartmouse::msgs::Maze &msg);
  void OnRobotCommand(const smartmouse::msgs::RobotCommand &msg);
  void OnRobotDescription(const smartmouse::msgs::RobotDescription &msg);

  void UpdateInternalState(double dt);
  void ResetRobot();
  void ResetTime();
  void PublishInternalState();
  void PublishWorldStats(Time rtf);

  ignition::transport::Node *node_ptr_;
  ignition::transport::Node::Publisher world_stats_pub_;
  ignition::transport::Node::Publisher sim_state_pub_;
  Time sim_time_;
  unsigned long steps_ = 0UL;
  std::mutex physics_mutex_;
  bool pause_ = true;
  bool quit_ = false;
  unsigned int ns_of_sim_per_step_ = 1000000u;
  unsigned long pause_at_steps_ = 0ul;
  double real_time_factor_ = 1.0;
  smartmouse::msgs::Maze maze_;
  smartmouse::msgs::RobotCommand cmd_;
  smartmouse::msgs::RobotDescription mouse_;

  smartmouse::msgs::InternalPhysicsState internal_state_;
};
