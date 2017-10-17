#include <limits>

#include <sim/simulator/lib/Server.h>
#include <lib/common/TopicNames.h>
#include <msgs/world_statistics.pb.h>
#include <common/core/Mouse.h>
#include <common/KinematicController/RobotConfig.h>
#include <common/math/math.h>
#include <lib/common/RayTracing.h>
#include <msgs/msgs.h>
#include <common/KinematicController/KinematicController.h>

Server::Server()
    : sim_time_(Time::Zero),
      steps_(0ul),
      pause_(true),
      quit_(false),
      connected_(false),
      ns_of_sim_per_step_(1000000u),
      pause_at_steps_(0),
      real_time_factor_(1) {
  ResetRobot(0.5, 0.5);
}

void Server::Start() {
  thread_ = new std::thread(std::bind(&Server::RunLoop, this));
}

void Server::Connect() {
  node_ptr_ = new ignition::transport::Node();
  world_stats_pub_ = node_ptr_->Advertise<smartmouse::msgs::WorldStatistics>(TopicNames::kWorldStatistics);
  sim_state_pub_ = node_ptr_->Advertise<smartmouse::msgs::RobotSimState>(TopicNames::kRobotSimState);
  node_ptr_->Subscribe(TopicNames::kServerControl, &Server::OnServerControl, this);
  node_ptr_->Subscribe(TopicNames::kPhysics, &Server::OnPhysics, this);
  node_ptr_->Subscribe(TopicNames::kMaze, &Server::OnMaze, this);
  node_ptr_->Subscribe(TopicNames::kRobotCommand, &Server::OnRobotCommand, this);
  node_ptr_->Subscribe(TopicNames::kRobotDescription, &Server::OnRobotDescription, this);
  connected_ = true;
}

void Server::RunLoop() {
  Connect();

  bool done = false;
  while (!done) {
    done = Run();
  }
}

bool Server::Run() {
  Time update_rate = Time(0, ns_of_sim_per_step_);
  Time start_step_time = Time::GetWallTime();
  Time desired_step_time = update_rate / real_time_factor_;
  Time desired_end_time = start_step_time + desired_step_time;

  // special case when update_rate is zero, like on startup.
  if (desired_step_time == 0) {
    Time::MSleep(1);
    return false;
  }

  // Begin Critical Section
  {
    std::lock_guard<std::mutex> guard(physics_mutex_);
    if (quit_) {
      return true;
    }

    if (pause_at_steps_ > 0 && pause_at_steps_ == steps_) {
      pause_at_steps_ = 0;
      pause_ = true;
      Time::MSleep(1);
      return false;
    }

    if (pause_) {
      Time::MSleep(1);
      return false;
    }

    Step();
  }
  // End Critical Section

  Time end_step_time = Time::GetWallTime();
  if (end_step_time > desired_end_time) {
    std::cout << "step took too long. Skipping sleep." << std::endl;
  }
  else {
    // FIXME: fudge factor makes sleep time more accurate, because we are often not woken up in time
    Time sleep_time = (desired_end_time - end_step_time) - 5e-5;
    Time::Sleep(sleep_time);
  }

  Time actual_end_step_time = Time::GetWallTime();
  double rtf = update_rate.Double() / (actual_end_step_time - start_step_time).Double();

  // This will send a message the GUI so it can update
  PublishInternalState();

  // announce completion of this step
  PublishWorldStats(rtf);

  return false;
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
  double col = robot_state_.p().col();
  double row = robot_state_.p().row();
  double theta = robot_state_.p().theta();
  double v_col = robot_state_.v().col();
  double v_row = robot_state_.v().row();
  double w = robot_state_.v().theta();
  double tl = robot_state_.left_wheel().theta();
  double wl = robot_state_.left_wheel().omega();
  double il = robot_state_.left_wheel().current();
  double tr = robot_state_.right_wheel().theta();
  double wr = robot_state_.right_wheel().omega();
  double ir = robot_state_.right_wheel().current();

  double vl = smartmouse::kc::radToMeters(wl);
  double vr = smartmouse::kc::radToMeters(wr);

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

  double dcol, drow, dtheta;
  double vl_cups = smartmouse::maze::toCellUnits(vl);
  double vr_cups = smartmouse::maze::toCellUnits(vr);
  std::tie(dcol, drow, dtheta) = KinematicController::forwardKinematics(vl_cups, vr_cups, theta, dt);

  // update row and col position
  double new_col = col + dcol;
  double new_row = row + drow;

  // update col and row speed and acceleration
  double new_v_col = (new_col - col) / dt;
  double new_v_row = (new_row - row) / dt;
  double new_a_col = (new_v_col - v_col) / dt;
  double new_a_row = (new_v_row - v_row) / dt;

  // update theta, omega, and alpha
  double new_theta = theta + dtheta; //eq 27
  double new_w = smartmouse::math::yawDiff(new_theta, theta) / dt;
  double new_a = (new_w - w) / dt;

  // handle wrap-around of theta
  smartmouse::math::wrapAngleRadInPlace(&new_theta);

  // Ray trace to find distance to walls
  // iterate over every line segment in the maze (all edges of all walls)
  // find the intersection of that wall with each sensor
  // if the intersection exists, and the distance is the shortest range for that sensor, replace the current range
  robot_state_.set_front(0.18);
//  robot_state_.set_front(ComputeSensorRange(mouse_.sensors().front()));
//  robot_state_.set_front_left(ComputeSensorRange(mouse_.sensors().front_left()));

  robot_state_.mutable_p()->set_col(new_col);
  robot_state_.mutable_p()->set_row(new_row);
  robot_state_.mutable_p()->set_theta(new_theta);
  robot_state_.mutable_v()->set_col(new_v_col);
  robot_state_.mutable_v()->set_row(new_v_row);
  robot_state_.mutable_v()->set_theta(new_w);
  robot_state_.mutable_a()->set_col(new_a_col);
  robot_state_.mutable_a()->set_row(new_a_row);
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

void Server::ResetRobot(double reset_col, double reset_row) {
  robot_state_.mutable_p()->set_col(reset_col);
  robot_state_.mutable_p()->set_row(reset_row);
  robot_state_.mutable_p()->set_theta(0);
  robot_state_.mutable_v()->set_col(0);
  robot_state_.mutable_v()->set_row(0);
  robot_state_.mutable_v()->set_theta(0);
  robot_state_.mutable_a()->set_col(0);
  robot_state_.mutable_a()->set_row(0);
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

void Server::PublishWorldStats(double rtf) {
  smartmouse::msgs::WorldStatistics world_stats_msg;
  world_stats_msg.set_steps(steps_);
  ignition::msgs::Time *sim_time_msg = world_stats_msg.mutable_sim_time();
  *sim_time_msg = sim_time_.toIgnMsg();
  world_stats_msg.set_real_time_factor(rtf);
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
      double reset_col = 0.5;
      double reset_row = 0.5;
      if (msg.has_reset_col()) {
        reset_col = msg.reset_col();
      }
      if (msg.has_reset_row()) {
        reset_row = msg.reset_row();
      }
      ResetRobot(reset_col, reset_row);
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

unsigned int Server::getNsOfSimPerStep() const {
  return ns_of_sim_per_step_;
}

double Server::ComputeSensorRange(smartmouse::msgs::SensorDescription sensor) {
  double range = std::numeric_limits<double>::max();
  std::vector<ignition::math::Line2d> maze_lines = smartmouse::msgs::MazeToLines(maze_);
  for (auto line : maze_lines) {
    ignition::math::Vector2d s_origin(sensor.p().x(), sensor.p().y());
    ignition::math::Vector2d s_direction(cos(sensor.p().theta()), sin(sensor.p().theta()));
    ignition::math::Vector2d pt;
    std::experimental::optional<double> r = RayTracing::distance_to_wall(line, s_origin, s_direction);
    if (r && *r < range) {
      range = *r;
    }
  }

  return range;
}
