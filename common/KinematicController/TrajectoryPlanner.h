#pragma  once

#include <cmath>

#include <common/Eigen/Eigen.h>
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

  const Eigen::Matrix<double, 10, 1> plan();

 private:
  Waypoints waypoints;

  // 6 constraints per row, x, y, xdot, ydot, trig, nonholonomic
  // 10 variables (two qunitic polynomials)
  Eigen::Matrix<double, Eigen::Dynamic, 10> A;
  Eigen::Matrix<double, 10, 1> b;
};

