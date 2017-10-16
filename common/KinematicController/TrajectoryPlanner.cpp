#include <common/KinematicController/TrajectoryPlanner.h>

TrajectoryPlanner::TrajectoryPlanner(Waypoints waypoints) : waypoints(waypoints) {
  for (Waypoint pt : waypoints) {
    A << x_constraint(pt.time),
        y_constraint(pt.time),
        non_holonomic_constraint(pt.state.pose.yaw, pt.time),
        trig_constraint(pt.state.pose.yaw, pt.time),
        x_dot_constraint(pt.time),
        y_dot_constraint(pt.time);
    b << pt.state.pose.col,
        pt.state.pose.row,
        0,
        pt.state.velocity * sin(2 * pt.state.pose.yaw);
  }

}

const Eigen::Matrix<double, 10, 1> TrajectoryPlanner::plan() {
  return A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
}

Eigen::Matrix<double, 1, 10> TrajectoryPlanner::x_constraint(double t) {
  Eigen::Matrix<double, 1, 10> vec;
  vec << 1, t, std::pow(t, 2), std::pow(t, 3), std::pow(t, 4), std::pow(t, 5), 0, 0, 0, 0, 0, 0;
  return vec;
}

Eigen::Matrix<double, 1, 10> TrajectoryPlanner::y_constraint(double t) {
  Eigen::Matrix<double, 1, 10> vec;
  vec << 0, 0, 0, 0, 0, 0, 1, t, std::pow(t, 2), std::pow(t, 3), std::pow(t, 4), std::pow(t, 5);
  return vec;
}

Eigen::Matrix<double, 1, 10> TrajectoryPlanner::non_holonomic_constraint(double theta_t, double t) {
  double s_t = sin(theta_t);
  double c_t = cos(theta_t);
  double t_2 = std::pow(t, 2);
  double t_3 = std::pow(t, 3);
  double t_4 = std::pow(t, 4);
  Eigen::Matrix<double, 1, 10> vec;
  vec << 0, s_t, 2 * s_t * t, 3 * s_t * t_2, 4 * s_t * t_3, 5 * s_t * t_4, 0, c_t, 2 * c_t * t, 3 * c_t * t_2, 4 * c_t
      * t_3, 5 * c_t * t_4;
  return vec;
}

Eigen::Matrix<double, 1, 10> TrajectoryPlanner::trig_constraint(double theta_t, double t) {
  double s_t = sin(theta_t);
  double c_t = cos(theta_t);
  double t_2 = std::pow(t, 2);
  double t_3 = std::pow(t, 3);
  double t_4 = std::pow(t, 4);
  Eigen::Matrix<double, 1, 10> vec;
  vec << 0, s_t, 2 * s_t * t, 3 * s_t * t_2, 4 * s_t * t_3, 5 * s_t * t_4, 0, -c_t, -2 * c_t * t, -3 * c_t * t_2, -4
      * c_t * t_3, -5 * c_t * t_4;
  return vec;
}

Eigen::Matrix<double, 1, 10> TrajectoryPlanner::x_dot_constraint(double t) {
  Eigen::Matrix<double, 1, 10> vec;
  vec << 0, 1, 2 * t, 3 * std::pow(t, 2), 4 * std::pow(t, 3), 5 * std::pow(t, 4), 0, 0, 0, 0, 0, 0;
  return vec;
}

Eigen::Matrix<double, 1, 10> TrajectoryPlanner::y_dot_constraint(double t) {
  Eigen::Matrix<double, 1, 10> vec;
  vec << 0, 0, 0, 0, 0, 0, 0, 1, 2 * t, 3 * std::pow(t, 2), 4 * std::pow(t, 3), 5 * std::pow(t, 4);
  return vec;
}
