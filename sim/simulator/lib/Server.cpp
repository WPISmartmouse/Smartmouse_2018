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
  node_ptr_->Subscribe(TopicNames::kRobotDescription, &Server::OnRobotDescription, this);

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

smartmouse::msgs::RobotSimState Server::UpdateInternalState(double dt) {
  const double J = mouse_.inertia();
  const double u_k = mouse_.u_kinetic();
  const double u_s = mouse_.u_static();

  // use the cmd abstract forces, apply our dynamics model, update internal state
  double x = internal_state_.p().x();
  double y = internal_state_.p().y();
  double theta = internal_state_.p().theta();
  double vx = internal_state_.v().x();
  double vy = internal_state_.v().y();
  double w = internal_state_.v().theta();
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
  double new_wl = wl + al * dt - u_k * wl;
  double new_wr = wr + ar * dt - u_k * wl;

  // TODO: model the speed of the motor given the input force
  double new_al = al;
  double new_ar = ar;
  if (cmd_.left().abstract_force() > u_s) {
    new_al = al + cmd_.left().abstract_force() - J*al;
    new_ar = ar + cmd_.right().abstract_force() - J*ar;
  }

  // forward kinematics for differential drive robot
  double R;
  double w_about_icc;
  double dtheta_about_icc;
  double new_x, new_y;

  // if we're going perfectly straight R is infinity, so check first.
  // update the x & y coordinates
  if (std::abs(vl-vr) > 1e-5) {
    R = config.TRACK_WIDTH * (vr + vl) / (2 *(vr - vl)); // eq 12
    w_about_icc = vl/(R-config.TRACK_WIDTH/2); //eq 11
    dtheta_about_icc = w_about_icc * dt; //eq 11
    new_x = x - R*(sin((vl-vr)/config.TRACK_WIDTH*dt-theta)+sin(theta)); //eq 28
    new_y = y - R*(cos((vl-vr)/config.TRACK_WIDTH*dt-theta)-cos(theta)); //eq 29
  }
  else {
    // going perfectly straight
    dtheta_about_icc = 0; //eq 11
    double v = (vl + vr) / 2;
    new_x = x + v * cos(theta) * dt;
    new_y = y + v * sin(theta) * dt;
  }

  double new_vx = (new_x - x) / dt;
  double new_vy = (new_y - y) / dt;
  double new_ax = (new_vx - vx) / dt;
  double new_ay = (new_vy - vy) / dt;

  // update theta, omega, and alpha
  double new_theta = theta + dtheta_about_icc; //eq 27
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
  sim_state_msg.set_left_wheel_velocity_mps(Mouse::radToMeters(internal_state_.left_wheel().omega()));
  sim_state_msg.set_right_wheel_velocity_mps(Mouse::radToMeters(internal_state_.right_wheel().omega()));

  return sim_state_msg;
}

void Server::ResetTime() {
  sim_time_ = Time::Zero;
  steps_ = 0UL;
  pause_at_steps_ = 0ul;
}

void Server::ResetRobot() {
  internal_state_.mutable_p()->set_x(0.09);
  internal_state_.mutable_p()->set_y(0.09);
  internal_state_.mutable_p()->set_theta(0);
  internal_state_.mutable_v()->set_x(0);
  internal_state_.mutable_v()->set_y(0);
  internal_state_.mutable_v()->set_theta(0);
  internal_state_.mutable_a()->set_x(0);
  internal_state_.mutable_a()->set_y(0);
  internal_state_.mutable_a()->set_theta(0);
  internal_state_.mutable_left_wheel()->set_theta(0);
  internal_state_.mutable_left_wheel()->set_omega(0);
  internal_state_.mutable_left_wheel()->set_alpha(0);
  internal_state_.mutable_right_wheel()->set_theta(0);
  internal_state_.mutable_right_wheel()->set_omega(0);
  internal_state_.mutable_right_wheel()->set_alpha(0);
  cmd_.mutable_left()->set_abstract_force(0);
  cmd_.mutable_right()->set_abstract_force(0);
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
    if (msg.has_reset_robot()) {
      ResetRobot();
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

void Server::OnRobotDescription(const smartmouse::msgs::RobotDescription &msg) {
  // Enter critical section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    mouse_ = msg;
  }
  // End critical section
}

void Server::join() {
  thread_->join();
}
