#pragma  once

#include <cmath>

#include <common/Eigen/Eigen/Dense>
#include <common/core/Pose.h>

struct Waypoint {
  double time;
  GlobalState state;
};

typedef std::vector<Waypoint> Waypoints;

class TrajectoryPlanner {

 public:
  static Eigen::Matrix<double, 1, 10> x_constraint(double t);

  static Eigen::Matrix<double, 1, 10> y_constraint(double t);

  static Eigen::Matrix<double, 1, 10> non_holonomic_constraint(double theta_t, double t);

  static Eigen::Matrix<double, 1, 10> trig_constraint(double theta_t, double t);

  static Eigen::Matrix<double, 1, 10> x_dot_constraint(double t);

  static Eigen::Matrix<double, 1, 10> y_dot_constraint(double t);

  TrajectoryPlanner(Waypoints waypoints);

 private:
  Waypoints waypoints;
};

