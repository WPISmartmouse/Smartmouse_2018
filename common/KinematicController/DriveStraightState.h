#pragma once

#include <common/core/Pose.h>
#include <common/core/Mouse.h>
#include "RobotConfig.h"

namespace smartmouse {
namespace kc {

extern const double kVf;

class DriveStraightState {
 public:
  DriveStraightState(GlobalPose start_pose,
                     double goalDisp,
                     double v_initial,
                     double v_final);

  std::pair<double, double> compute_wheel_velocities(Mouse &mouse, double t_s);

  constexpr static double kPWall = 0.8;
  constexpr static double kPYaw = -1.2;

  double dispError();

  double compute_forward_velocity(double t_s);

 private:
  static constexpr double a_m = 2; // cu/s^2
  static constexpr double j_m = 10; // cu/s^3
  double goal_disp = 0;
  const double v_0 = 0;
  double v_f = 0;
  const double t_1 = 0;
  const double v_1 = 0;
  const double v_2 = 0;
  double t_2 = 0;
  double t_f = 0;
  double disp = 0;
  GlobalPose start_pose;

};

}
}


