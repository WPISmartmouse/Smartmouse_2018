#pragma once

namespace smartmouse {
namespace kc {

class VelocityProfileTiming {

 public:
  VelocityProfileTiming(double d, double v_0, double v_f);

  double compute_forward_velocity(double t) const;

  static constexpr double a_m = 5; // cu/s^2
  static constexpr double j_m = 60; // cu/s^3

  const double d;
  const double v_0;
  const double v_f;

  double t_1;
  double t_2;
  double t_m1;
  double t_m2;
  double t_3;
  double t_4;
  double t_f;

  double v_m;
  double v_1;
  double v_2;
  double v_3;
  double v_4;

  double profile_distance(double v_0, double v_f, double a_m, double j_m, double v_m) const;

  double compute_v_m(double v_0, double v_f, double a_m, double j_m, double d) const;

  double three_phase_profile_d(double v_a, double v_b) const;

  double two_phase_stop_profile_d(double v_begin) const;
  double two_phase_start_profile_d(double v_end) const;
};


}
}
