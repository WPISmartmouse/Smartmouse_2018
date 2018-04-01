#include <limits>

#include <common/KinematicController/RobotConfig.h>
#include <common/math/math.h>
#include <common/KinematicController/KinematicController.h>
#include <sim/simulator/lib/common/RayTracing.h>
#include <sim/simulator/lib/Server.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <sim/simulator/msgs/world_statistics.pb.h>
#include <common/IRSensorModeling/Model.h>

Server::Server()
    : sim_time_(Time::Zero),
      steps_(0ul),
      pause_(true),
      static_(false),
      quit_(false),
      connected_(false),
      ns_of_sim_per_step_(1000000u),
      pause_at_steps_(0),
      real_time_factor_(1),
      mouse_set_(false),
      max_cells_to_check_(0) {
  ResetRobot(0.5, 0.5, 0);
}

void Server::Start() {
  thread_ = new std::thread(std::bind(&Server::RunLoop, this));
}

void Server::Connect() {
  node_ptr_ = new ignition::transport::Node();
  world_stats_pub_ = node_ptr_->Advertise<smartmouse::msgs::WorldStatistics>(TopicNames::kWorldStatistics);
  sim_state_pub_ = node_ptr_->Advertise<smartmouse::msgs::RobotSimState>(TopicNames::kRobotSimState);
  server_control_pub_ = node_ptr_->Advertise<smartmouse::msgs::ServerControl>(TopicNames::kServerControl);
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
//    std::lock_guard<std::mutex> guard(physics_mutex_);
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
    // FIXME: do proper logging control
    // std::cout << "step took too long. Skipping sleep." << std::endl;
  } else {
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

  if (mouse_set_) {
    UpdateRobotState(dt.Double());
  }

  // increment step counter
  ++steps_;
}

void Server::UpdateRobotState(double dt) {
  // TODO: Implement friction
  const double u_k = mouse_.motor().u_kinetic();
  const double u_s = mouse_.motor().u_static();

  const double motor_J = mouse_.motor().j();
  const double motor_b = mouse_.motor().b();
  const double motor_K = mouse_.motor().k();
  const double motor_R = mouse_.motor().r();
  const double motor_L = mouse_.motor().l();

  // use the cmd abstract forces, apply our dynamics model, update robot state
  double col = robot_state_.p().col();
  double row = robot_state_.p().row();
  double yaw = robot_state_.p().yaw();
  double v_col = robot_state_.v().col();
  double v_row = robot_state_.v().row();
  double dyawdt = robot_state_.v().yaw();
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
//  if (wl < 1e-3 && new_al < u_s) {
//    new_al = 0;
//  }
//  else if (new_al < u_k) {
//    new_al = 0;
//  }
//  if (wr < 1e-3 && new_ar < u_s) {
//    new_ar = 0;
//  }
//  else if (new_ar < u_k) {
//    new_ar = 0;
//  }

  double new_wl = wl + new_al * dt;
  double new_wr = wr + new_ar * dt;
  double new_tl = tl + new_wl * dt + 1 / 2 * new_al * dt * dt;
  double new_tr = tr + new_wr * dt + 1 / 2 * new_ar * dt * dt;

  const double kVRef = 5.0;
  double voltage_l = (cmd_.left().abstract_force() * kVRef) / 255.0;
  double voltage_r = (cmd_.right().abstract_force() * kVRef) / 255.0;
  double new_il = il + dt * (voltage_l - motor_K * wl - motor_R * il) / motor_L;
  double new_ir = ir + dt * (voltage_r - motor_K * wr - motor_R * ir) / motor_L;

  double vl_cups = smartmouse::maze::toCellUnits(vl);
  double vr_cups = smartmouse::maze::toCellUnits(vr);
  GlobalPose d_pose = KinematicController::forwardKinematics(vl_cups, vr_cups, yaw, dt);

  // update row and col position
  double new_col = col + d_pose.col;
  double new_row = row + d_pose.row;

  // update col and row speed and acceleration
  double new_v_col = (new_col - col) / dt;
  double new_v_row = (new_row - row) / dt;
  double new_a_col = (new_v_col - v_col) / dt;
  double new_a_row = (new_v_row - v_row) / dt;

  // update yaw, omega, and alpha
  double new_yaw = yaw + d_pose.yaw; //eq 27
  double new_dyawdt = smartmouse::math::yaw_diff(new_yaw, yaw) / dt;
  double new_a = (new_dyawdt - dyawdt) / dt;

  // handle wrap-around of theta
  smartmouse::math::wrapAngleRadInPlace(&new_yaw);

  // Ray trace to find distance to walls
  // iterate over every line segment in the maze (all edges of all walls)
  // find the intersection of that wall with each sensor
  // if the intersection exists, and the distance is the shortest range for that sensor, replace the current range
  auto stamp = robot_state_.mutable_stamp();
  *stamp = sim_time_.toIgnMsg();
  robot_state_.set_front_m(ComputeSensorDistToWall(mouse_.sensors().front()));
  robot_state_.set_front_left_m(ComputeSensorDistToWall(mouse_.sensors().front_left()));
  robot_state_.set_front_right_m(ComputeSensorDistToWall(mouse_.sensors().front_right()));
  robot_state_.set_gerald_left_m(ComputeSensorDistToWall(mouse_.sensors().gerald_left()));
  robot_state_.set_gerald_right_m(ComputeSensorDistToWall(mouse_.sensors().gerald_right()));
  robot_state_.set_back_left_m(ComputeSensorDistToWall(mouse_.sensors().back_left()));
  robot_state_.set_back_right_m(ComputeSensorDistToWall(mouse_.sensors().back_right()));

  robot_state_.set_front_adc(ComputeADCValue(mouse_.sensors().front()));
  robot_state_.set_front_left_adc(ComputeADCValue(mouse_.sensors().front_left()));
  robot_state_.set_front_right_adc(ComputeADCValue(mouse_.sensors().front_right()));
  robot_state_.set_gerald_left_adc(ComputeADCValue(mouse_.sensors().gerald_left()));
  robot_state_.set_gerald_right_adc(ComputeADCValue(mouse_.sensors().gerald_right()));
  robot_state_.set_back_left_adc(ComputeADCValue(mouse_.sensors().back_left()));
  robot_state_.set_back_right_adc(ComputeADCValue(mouse_.sensors().back_right()));

  if (!static_) {
    robot_state_.mutable_p()->set_col(new_col);
    robot_state_.mutable_p()->set_row(new_row);
    robot_state_.mutable_p()->set_yaw(new_yaw);
    robot_state_.mutable_v()->set_col(new_v_col);
    robot_state_.mutable_v()->set_row(new_v_row);
    robot_state_.mutable_v()->set_yaw(new_dyawdt);
    robot_state_.mutable_a()->set_col(new_a_col);
    robot_state_.mutable_a()->set_row(new_a_row);
    robot_state_.mutable_a()->set_yaw(new_a);
  }
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

void Server::ResetRobot(double reset_col, double reset_row, double reset_yaw) {
  robot_state_.mutable_p()->set_col(reset_col);
  robot_state_.mutable_p()->set_row(reset_row);
  robot_state_.mutable_p()->set_yaw(reset_yaw);
  robot_state_.mutable_v()->set_col(0);
  robot_state_.mutable_v()->set_row(0);
  robot_state_.mutable_v()->set_yaw(0);
  robot_state_.mutable_a()->set_col(0);
  robot_state_.mutable_a()->set_row(0);
  robot_state_.mutable_a()->set_yaw(0);
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
//    std::lock_guard<std::mutex> guard(physics_mutex_);
    if (msg.has_pause()) {
      pause_ = msg.pause();
    }
    else if (msg.toggle_play_pause()) {
      smartmouse::msgs::ServerControl play_pause_msg;
      play_pause_msg.set_pause(!pause_);
      server_control_pub_.Publish(play_pause_msg);
    }
    if (msg.has_static_()) {
      static_ = msg.static_();
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
      double reset_yaw = 0;
      if (msg.has_reset_col()) {
        reset_col = msg.reset_col();
      }
      if (msg.has_reset_row()) {
        reset_row = msg.reset_row();
      }
      if (msg.has_reset_yaw()) {
        reset_yaw = msg.reset_yaw();
      }
      ResetRobot(reset_col, reset_row, reset_yaw);
    }
  }
  // End critical section
}

void Server::OnPhysics(const smartmouse::msgs::PhysicsConfig &msg) {
  // Enter critical section
  {
//    std::lock_guard<std::mutex> guard(physics_mutex_);
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
//    std::lock_guard<std::mutex> guard(physics_mutex_);
    smartmouse::msgs::Convert(msg, maze_walls_);
  }
  // End critical section
}

void Server::OnRobotCommand(const smartmouse::msgs::RobotCommand &msg) {
  // Enter critical section
  {
//    std::lock_guard<std::mutex> guard(physics_mutex_);
    cmd_ = msg;
  }
  // End critical section
}

void Server::OnRobotDescription(const smartmouse::msgs::RobotDescription &msg) {
  // Enter critical section
  {
//    std::lock_guard<std::mutex> guard(physics_mutex_);
    mouse_ = msg;
    ComputeMaxSensorRange();
    mouse_set_ = true;
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

int Server::ComputeADCValue(smartmouse::msgs::SensorDescription sensor) {
  // take the actual distance and the angle and reverse-calculate the ADC value
  double d = ComputeSensorDistToWall(sensor);
  smartmouse::ir::ModelParams model{sensor.a(), sensor.b(), sensor.c(), sensor.d()};
  return model.toADC(d);
}

double Server::ComputeSensorDistToWall(smartmouse::msgs::SensorDescription sensor) {
  double min_range = smartmouse::kc::ANALOG_MAX_DIST_CU;
  double sensor_col = smartmouse::maze::toCellUnits(sensor.p().x());
  double sensor_row = smartmouse::maze::toCellUnits(sensor.p().y());
  double robot_theta = robot_state_.p().yaw();
  ignition::math::Vector3d s_origin_3d(sensor_col, sensor_row, 1);
  ignition::math::Matrix3d tf(cos(robot_theta), -sin(robot_theta), robot_state_.p().col(),
                              sin(robot_theta), cos(robot_theta), robot_state_.p().row(),
                              0, 0, 1);
  s_origin_3d = tf * s_origin_3d;
  ignition::math::Vector2d s_origin(s_origin_3d.X(), s_origin_3d.Y());
  ignition::math::Vector2d s_direction(cos(robot_theta + sensor.p().theta()), sin(robot_theta + sensor.p().theta()));

  // iterate over the lines of walls that are nearby
  int row = (int) robot_state_.p().row();
  int col = (int) robot_state_.p().col();
  unsigned int min_r = (unsigned int) std::max(0, row - (int) max_cells_to_check_);
  unsigned int max_r = std::min(smartmouse::maze::SIZE, row + max_cells_to_check_);
  unsigned int min_c = (unsigned int) std::max(0, col - (int) max_cells_to_check_);
  unsigned int max_c = std::min(smartmouse::maze::SIZE, col + max_cells_to_check_);
  for (unsigned int r = min_r; r < max_r; r++) {
    for (unsigned int c = min_c; c < max_c; c++) {
      // get the walls at r/c
      for (auto wall : maze_walls_[r][c]) {
        std::vector<ignition::math::Line2d> wall_lines_;
        wall_lines_.push_back(ignition::math::Line2d(wall.c1(), wall.r1(), wall.c1(), wall.r2()));
        wall_lines_.push_back(ignition::math::Line2d(wall.c1(), wall.r2(), wall.c2(), wall.r2()));
        wall_lines_.push_back(ignition::math::Line2d(wall.c2(), wall.r2(), wall.c2(), wall.r1()));
        wall_lines_.push_back(ignition::math::Line2d(wall.c2(), wall.r1(), wall.c1(), wall.r1()));
        for (auto line : wall_lines_) {
          std::experimental::optional<double> range = RayTracing::distance_to_wall(line, s_origin, s_direction);
          if (range && *range < min_range) {
            min_range = *range;
          }
        }
      }
    }
  }

  if (min_range < smartmouse::kc::ANALOG_MIN_DIST_CU) {
    min_range = smartmouse::kc::ANALOG_MIN_DIST_CU;
  }

  return smartmouse::maze::toMeters(min_range);
}

void Server::ComputeMaxSensorRange() {
  double max_range = 0;

  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().front()));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().front_left()));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().front_right()));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().gerald_left()));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().gerald_right()));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().back_left()));
  max_range = std::max(max_range, ComputeSensorRange(mouse_.sensors().back_right()));

  max_cells_to_check_ = (unsigned int) std::ceil(smartmouse::maze::toCellUnits(max_range));
}

const double Server::ComputeSensorRange(const smartmouse::msgs::SensorDescription sensor) {
  double sensor_x = sensor.p().x();
  double sensor_y = sensor.p().x();
  double range_x = sensor_x + cos(sensor.p().theta()) * smartmouse::kc::ANALOG_MAX_DIST_M;
  double range_y = sensor_y + sin(sensor.p().theta()) * smartmouse::kc::ANALOG_MAX_DIST_M;
  return std::hypot(range_x, range_y);
}
