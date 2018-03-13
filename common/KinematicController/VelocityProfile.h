#pragma once

#include <common/core/Pose.h>
#include <common/core/Mouse.h>
#include "RobotConfig.h"

namespace smartmouse {
namespace kc {

extern const double kVf;

class VelocityProfile {
 public:

  VelocityProfile(GlobalPose start_pose,
                     double goal_disp,
                     double v_initial,
                     double v_final);

  std::pair<double, double> drive_straight_wheel_velocities(Mouse &mouse, double t_s);

  constexpr static double kPWall = 0.8;
  constexpr static double kPYaw = -1.2;

  double dispError();

  double compute_forward_velocity(double t_s);

 private:
  static constexpr double a_m = 5; // cu/s^2
  static constexpr double j_m = 50; // cu/s^3
  const double d;
  const double v_0;
  const double v_f;
  const double v_m;
  const double t_1;
  const double v_1;
  const double v_2;
  const double t_2;
  const double t_m1;
  const double t_m2;
  const double t_3;
  const double v_3;
  const double v_4;
  const double t_4;
  const double t_f;
  double disp = 0;
  GlobalPose start_pose;

};

}
}


