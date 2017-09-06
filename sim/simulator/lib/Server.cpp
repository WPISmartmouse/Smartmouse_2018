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
  node_ptr_->Subscribe(TopicNames::kMaze, &Server::OnMaze, this);

  while (true) {
    Time update_rate = Time(0, ns_of_sim_per_step_);
    Time start_step_time = Time::GetWallTime();
    Time desired_step_time = update_rate / real_time_factor_;

    // special case when update_rate is zero, like on startup.
    if (desired_step_time == 0) {
      Time::MSleep(1);
      continue;
    }

    // Begin Critical Section
    {
      std::lock_guard<std::mutex> guard(physics_mutex_);
      if (quit_) {
        break;
      }

      if (pause_at_steps_ > 0 && pause_at_steps_ == steps_) {
        pause_at_steps_ = 0;
        pause_ = true;
        Time::MSleep(1);
        continue;
      }

      if (pause_) {
        Time::MSleep(1);
        continue;
      }

      Step();
    }
    // End Critical Section

    Time end_step_time = Time::GetWallTime();
    Time used_step_time = end_step_time - start_step_time;

    if (used_step_time < desired_step_time) {
      // there is a fudge factor here to account for the time to publish world stats
      Time sleep_time = desired_step_time - used_step_time - 50e-6;
      Time::Sleep(sleep_time);
    }
    else {
      std::cout << "Update took " << used_step_time.Float() << ". Skipping sleep." << std::endl;
    }

    Time actual_end_step_time = Time::GetWallTime();
    Time actual_step_time = actual_end_step_time - start_step_time;
    Time rtf = update_rate / actual_step_time;

    // announce completion of this step
    smartmouse::msgs::WorldStatistics world_stats_msg;
    world_stats_msg.set_steps(steps_);
    ignition::msgs::Time *sim_time_msg = world_stats_msg.mutable_sim_time();
    *sim_time_msg = sim_time_.toIgnMsg();
    world_stats_msg.set_real_time_factor(rtf.Double());
    world_stats_pub_.Publish(world_stats_msg);
  }
}

void Server::Step() {
  // update sim time
  sim_time_ += Time(0, ns_of_sim_per_step_);

  // increment step counter
  ++steps_;
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
    if (msg.has_ns_of_sim_per_step()) {
      ns_of_sim_per_step_ = msg.ns_of_sim_per_step();
    }
    if (msg.has_real_time_factor()) {
      if (msg.real_time_factor() >= 1e-3 && msg.real_time_factor() <= 10) {
        real_time_factor_ = msg.real_time_factor();
      }
    }
  }
  // End critical section
}

void Server::OnMaze(const smartmouse::msgs::Maze &msg) {
  // Enter critical section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    maze_ = msg;
  }
  // End critical section
}

void Server::join() {
  thread_->join();
}
