#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/TopicNames.h>
#include <msgs/world_statistics.pb.h>
#include <common/Mouse.h>
#include <common/RobotConfig.h>

void Server::start() {
  thread_ = new std::thread(std::bind(&Server::RunLoop, this));
  sim_time_ = Time::Zero;
}

void Server::RunLoop() {
  node_ptr_ = new ignition::transport::Node();

  world_stats_pub_ = node_ptr_->Advertise<smartmouse::msgs::WorldStatistics>(TopicNames::kWorldStatistics);
  sim_state_pub_ = node_ptr_->Advertise<smartmouse::msgs::RobotSimState>(TopicNames::kRobotSimState);
  node_ptr_->Subscribe(TopicNames::kServerControl, &Server::OnServerControl, this);
  node_ptr_->Subscribe(TopicNames::kPhysics, &Server::OnPhysics, this);
  node_ptr_->Subscribe(TopicNames::kMaze, &Server::OnMaze, this);
  node_ptr_->Subscribe(TopicNames::kRobotCommand, &Server::OnRobotCommand, this);

  while (true) {
    Time update_rate = Time(0, ns_of_sim_per_step_);
    Time start_step_time = Time::GetWallTime();
    Time desired_step_time = update_rate / real_time_factor_;

    // special case when update_rate is zero, like on startup.
    if (desired_step_time == 0) {
      Time::MSleep(1);
      continue;
    }

    smartmouse::msgs::RobotSimState sim_state_msg;

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

      sim_state_msg = Step();
    }
    // End Critical Section

    Time end_step_time = Time::GetWallTime();
    Time used_step_time = end_step_time - start_step_time;

    if (used_step_time < desired_step_time) {
      // there is a fudge factor here to account for the time to publish world stats
      Time sleep_time = desired_step_time - used_step_time - 50e-6;
      Time::Sleep(sleep_time);
    } else {
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

    sim_state_pub_.Publish(sim_state_msg);
  }
}

smartmouse::msgs::RobotSimState Server::Step() {
  // update sim time
  auto dt = Time(0, ns_of_sim_per_step_);
  sim_time_ += dt;

  auto sim_state_msg = UpdateInternalState(dt.Double());

  // increment step counter
  ++steps_;

  return sim_state_msg;
}

void Server::ResetTime() {
  sim_time_ = Time::Zero;
  steps_ = 0UL;
  pause_at_steps_ = 0ul;
}

void Server::OnServerControl(const smartmouse::msgs::ServerControl &msg) {
  // Enter critical section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    if (msg.has_pause()) {
      pause_ = msg.pause();
    }
    if (msg.has_quit()) {
      quit_ = msg.quit();
    }
    if (msg.has_step()) {
      pause_ = false;
      pause_at_steps_ = steps_ + msg.step();
    }
    if (msg.has_reset_time()) {
      ResetTime();
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

void Server::OnRobotCommand(const smartmouse::msgs::RobotCommand &msg) {
  // Enter critical section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    cmd_ = msg;
  }
  // End critical section
}

void Server::join() {
  thread_->join();
}

smartmouse::msgs::RobotSimState Server::UpdateInternalState(double dt) {
  // use the cmd abstract forces, apply our dynamics model, update internal state
  double x = internal_state_.p().x();
  double y = internal_state_.p().y();
  double theta = internal_state_.p().theta();
  double vx = internal_state_.v().x();
  double vy = internal_state_.v().y();
  double w = internal_state_.v().theta();
  double ax = internal_state_.a().x();
  double ay = internal_state_.a().y();
  double a = internal_state_.a().theta();
  double tl = internal_state_.left_wheel().theta();
  double wl = internal_state_.left_wheel().omega();
  double al = internal_state_.left_wheel().alpha();
  double tr = internal_state_.right_wheel().theta();
  double wr = internal_state_.right_wheel().omega();
  double ar = internal_state_.right_wheel().alpha();

  double vl = wl * (M_2_PI * Mouse::WHEEL_RAD);
  double vr = wr * (M_2_PI * Mouse::WHEEL_RAD);

  // newtons equations of rotational motion to update wheel states
  double new_tl = tl + wl * dt + 1/2 * al * dt * dt;
  double new_tr = tr + wr * dt + 1/2 * ar * dt * dt;
  double new_wl = wl + al;
  double new_wr = wr + ar;
  double new_al = al + cmd_.left().abstract_force();
  double new_ar = ar + cmd_.right().abstract_force();

  // forward kinematics for differential drive robot
  double w_about_icc = 0.5 * wl + 0.5 * wl;
  double dtheta_about_icc = w_about_icc * dt;
  double R = 0;
  double new_x, new_y;
  // if we're going perfectly straight R is infinity, so check first.
  // update the x & y coordinates
  if (std::abs(vl-vr) < 1e-5) {
    R = config.TRACK_WIDTH * (vr + vl) / (2 *(vr - vl));
    // TODO: update x & y
    new_x = x + 0;
    new_y = y + 0;
  }
  else {
    double v = (vl + vr) / 2;
    new_x = x + cos(theta) * dt * v;
    new_y = y + sin(theta) * dt * v;
  }

  double new_vx = (new_x - x) / dt;
  double new_vy = (new_y - y) / dt;
  double new_ax = (new_vx - vx) / dt;
  double new_ay = (new_vy - vy) / dt;

  // update theta, omega, and alpha
  double new_theta = theta + dtheta_about_icc;
  double new_w = (new_theta - theta) / dt;
  double new_a = (new_w - w) / dt;

  internal_state_.mutable_p()->set_x(new_x);
  internal_state_.mutable_p()->set_y(new_y);
  internal_state_.mutable_p()->set_theta(new_theta);
  internal_state_.mutable_v()->set_x(new_vx);
  internal_state_.mutable_v()->set_y(new_vy);
  internal_state_.mutable_v()->set_theta(new_w);
  internal_state_.mutable_a()->set_x(new_ax);
  internal_state_.mutable_a()->set_y(new_ay);
  internal_state_.mutable_a()->set_theta(new_a);
  internal_state_.mutable_left_wheel()->set_theta(new_tl);
  internal_state_.mutable_left_wheel()->set_omega(new_wl);
  internal_state_.mutable_left_wheel()->set_alpha(new_al);
  internal_state_.mutable_right_wheel()->set_theta(new_tr);
  internal_state_.mutable_right_wheel()->set_omega(new_wr);
  internal_state_.mutable_right_wheel()->set_alpha(new_ar);

  smartmouse::msgs::RobotSimState sim_state_msg;
  auto stamp = sim_state_msg.mutable_stamp();
  stamp->set_sec(sim_time_.sec);
  stamp->set_nsec(sim_time_.nsec);
  sim_state_msg.set_true_x_meters(internal_state_.p().x());
  sim_state_msg.set_true_y_meters(internal_state_.p().y());
  sim_state_msg.set_true_yaw_rad(internal_state_.p().theta());

  return sim_state_msg;
}
