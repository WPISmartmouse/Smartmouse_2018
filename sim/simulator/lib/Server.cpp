#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/TopicNames.h>
#include <msgs/world_statistics.pb.h>
#include <common/Mouse.h>
#include <common/RobotConfig.h>
#include <common/math/math.h>

void Server::Start() {
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
  connected_ = true;

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
    } else {
      std::cout << "Update took " << used_step_time.Float() << ". Skipping sleep." << std::endl;
    }

    Time actual_end_step_time = Time::GetWallTime();
    Time actual_step_time = actual_end_step_time - start_step_time;
    Time rtf = update_rate / actual_step_time;

    // This will send a message the GUI so it can update
    PublishInternalState();

    // announce completion of this step
    PublishWorldStats(rtf);
  }
}

void Server::Step() {
  // update sim time
  auto dt = Time(0, ns_of_sim_per_step_);
  sim_time_ += dt;

  UpdateRobotState(dt.Double());

  // increment step counter
  ++steps_;
}

void Server::UpdateRobotState(double dt) {
  // TODO: implement friction
//  const double u_k = mouse_.motor().u_kinetic();
//  const double u_s = mouse_.motor().u_static();
  const double motor_J = mouse_.motor().j();
  const double motor_b = mouse_.motor().b();
  const double motor_K = mouse_.motor().k();
  const double motor_R = mouse_.motor().r();
  const double motor_L = mouse_.motor().l();

  // use the cmd abstract forces, apply our dynamics model, update robot state
  double x = robot_state_.p().x();
  double y = robot_state_.p().y();
  double theta = robot_state_.p().theta();
  double vx = robot_state_.v().x();
  double vy = robot_state_.v().y();
  double w = robot_state_.v().theta();
  double tl = robot_state_.left_wheel().theta();
  double wl = robot_state_.left_wheel().omega();
  double il = robot_state_.left_wheel().current();
  double tr = robot_state_.right_wheel().theta();
  double wr = robot_state_.right_wheel().omega();
  double ir = robot_state_.right_wheel().current();

  double vl = Mouse::radToMeters(wl);
  double vr = Mouse::radToMeters(wr);

  double new_al = il * motor_K / motor_J - motor_b * wl / motor_J; // equation 36
  double new_ar = ir * motor_K / motor_J - motor_b * wr / motor_J; // equation 39
  double new_wl = wl + new_al * dt;
  double new_wr = wr + new_ar * dt;
  double new_tl = tl + new_wl * dt + 1 / 2 * new_al * dt * dt;
  double new_tr = tr + new_wr * dt + 1 / 2 * new_ar * dt * dt;

  const double kVRef = 5.0;
  double voltage_l = (cmd_.left().abstract_force() * kVRef) / 255.0;
  double voltage_r = (cmd_.right().abstract_force() * kVRef) / 255.0;
  double new_il = il + dt * (voltage_l - motor_K * wl - motor_R * il) / motor_L;
  double new_ir = ir + dt * (voltage_r - motor_K * wr - motor_R * ir) / motor_L;

  // forward kinematics for differential drive robot
  double R = 0;
  double w_about_icc = 0;
  double dtheta_about_icc = 0;
  double new_x = 0, new_y = 0;

  // if we're going perfectly straight R is infinity, so check first.
  // update the x & y coordinates
  if (std::abs(vl - vr) > 1e-5) {
    R = config.TRACK_WIDTH * (vr + vl) / (2 * (vr - vl)); // eq 12
    w_about_icc = (vr - vl) / config.TRACK_WIDTH; //eq 11
    dtheta_about_icc = w_about_icc * dt; //eq 11
    new_x = x + R * (sin(dtheta_about_icc + theta) - sin(theta)); //eq 28
    new_y = y - R * (cos(dtheta_about_icc + theta) - cos(theta)); //eq 29
  } else {
    // going perfectly straight
    dtheta_about_icc = 0;
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
  double new_w = smartmouse::math::yawDiff(new_theta, theta) / dt;
  double new_a = (new_w - w) / dt;

  // handle wrap-around of theta
  new_theta = smartmouse::math::wrapAngleRad(new_theta);

  robot_state_.mutable_p()->set_x(new_x);
  robot_state_.mutable_p()->set_y(new_y);
  robot_state_.mutable_p()->set_theta(new_theta);
  robot_state_.mutable_v()->set_x(new_vx);
  robot_state_.mutable_v()->set_y(new_vy);
  robot_state_.mutable_v()->set_theta(new_w);
  robot_state_.mutable_a()->set_x(new_ax);
  robot_state_.mutable_a()->set_y(new_ay);
  robot_state_.mutable_a()->set_theta(new_a);
  robot_state_.mutable_left_wheel()->set_theta(new_tl);
  robot_state_.mutable_left_wheel()->set_omega(new_wl);
  robot_state_.mutable_left_wheel()->set_alpha(new_al);
  robot_state_.mutable_left_wheel()->set_current(new_il);
  robot_state_.mutable_right_wheel()->set_theta(new_tr);
  robot_state_.mutable_right_wheel()->set_omega(new_wr);
  robot_state_.mutable_right_wheel()->set_alpha(new_ar);
  robot_state_.mutable_right_wheel()->set_current(new_ir);
}

void Server::ResetTime() {
  sim_time_ = Time::Zero;
  steps_ = 0UL;
  pause_at_steps_ = 0ul;

  PublishInternalState();
  PublishWorldStats(0);
}

void Server::ResetRobot() {
  robot_state_.mutable_p()->set_x(0.09);
  robot_state_.mutable_p()->set_y(0.09);
  robot_state_.mutable_p()->set_theta(0);
  robot_state_.mutable_v()->set_x(0);
  robot_state_.mutable_v()->set_y(0);
  robot_state_.mutable_v()->set_theta(0);
  robot_state_.mutable_a()->set_x(0);
  robot_state_.mutable_a()->set_y(0);
  robot_state_.mutable_a()->set_theta(0);
  robot_state_.mutable_left_wheel()->set_theta(0);
  robot_state_.mutable_left_wheel()->set_omega(0);
  robot_state_.mutable_left_wheel()->set_current(0);
  robot_state_.mutable_right_wheel()->set_theta(0);
  robot_state_.mutable_right_wheel()->set_omega(0);
  robot_state_.mutable_right_wheel()->set_current(0);
  cmd_.mutable_left()->set_abstract_force(0);
  cmd_.mutable_right()->set_abstract_force(0);

  PublishInternalState();
}

void Server::PublishInternalState() {
  sim_state_pub_.Publish(robot_state_);
}

void Server::PublishWorldStats(Time rtf) {
  smartmouse::msgs::WorldStatistics world_stats_msg;
  world_stats_msg.set_steps(steps_);
  ignition::msgs::Time *sim_time_msg = world_stats_msg.mutable_sim_time();
  *sim_time_msg = sim_time_.toIgnMsg();
  world_stats_msg.set_real_time_factor(rtf.Double());
  world_stats_pub_.Publish(world_stats_msg);
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

void Server::Join() {
  thread_->join();
}

bool Server::IsConnected() {
  return connected_;
}
