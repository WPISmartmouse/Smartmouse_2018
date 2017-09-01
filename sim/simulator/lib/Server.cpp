#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/TopicNames.h>
#include <msgs/world_statistics.pb.h>

void Server::start() {
  thread_ = new std::thread(std::bind(&Server::RunLoop, this));
  sim_time_ = Time::Zero;
}

void Server::RunLoop() {
  node_ptr_ = new ignition::transport::Node();

  world_stats_pub_ = node_ptr_->Advertise<smartmouse::msgs::WorldStatistics>(TopicNames::kWorldStatistics);
  node_ptr_->Subscribe(TopicNames::kWorldControl, &Server::OnWorldControl, this);
  node_ptr_->Subscribe(TopicNames::kPhysics, &Server::OnPhysics, this);

  while (true) {
    Time update_rate = Time(0, ns_per_step_);
    Time expected_end_step_time = Time::GetWallTime();
    expected_end_step_time += update_rate;

    // Begin Critical Section
    {
      std::lock_guard<std::mutex> guard(physics_mutex_);
      if (quit_) {
        break;
      }

      Step();
    }
    // End Critical Section

    Time end_step_time = Time::GetWallTime();
    if (end_step_time < expected_end_step_time) {
      Time sleep_time = expected_end_step_time - end_step_time;
      Time::Sleep(sleep_time);
    }
  }
}

void Server::Step() {
  if (pause_at_steps_ > 0 && pause_at_steps_ == steps_) {

    pause_at_steps_ = 0;
    pause_ = true;
    return;
  }

  if (pause_) {
    return;
  }

  // update sim time
  sim_time_ += Time(0, ns_per_step_);

  // increment step counter
  ++steps_;

  // announce completion of this step
  smartmouse::msgs::WorldStatistics world_stats_msg;
  world_stats_msg.set_steps(steps_);
  ignition::msgs::Time *sim_time_msg = world_stats_msg.mutable_sim_time();
  *sim_time_msg = sim_time_.toIgnMsg();
  world_stats_pub_.Publish(world_stats_msg);
}

void Server::OnWorldControl(const smartmouse::msgs::ServerControl &msg) {
  // Enter critical section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    pause_ = msg.pause();
    quit_ = msg.quit();
    if (msg.has_step()) {
      pause_at_steps_ = steps_ + msg.step();
    }
  }
  // End critical section
}

void Server::OnPhysics(const smartmouse::msgs::PhysicsConfig &msg) {
  // Enter critical section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    ns_per_step_ = msg.ns_per_step();
  }
  // End critical section
}

void Server::join() {
  thread_->join();
}
