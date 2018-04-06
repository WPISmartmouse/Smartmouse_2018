#include <algorithm>

#include <common/KinematicController/VelocityProfileTiming.h>
#include "RobotConfig.h"

namespace smartmouse {
namespace kc {

double VelocityProfileTiming::profile_distance(double v_0, double v_f, double a_m, double j_m, double v_m) const {
  return (std::pow(a_m, 2) * (v_0 + v_f + 2 * v_m) - j_m * (std::pow(v_0, 2) + std::pow(v_f, 2) - 2 * std::pow(v_m, 2)))
      / (2 * a_m * j_m);
}

double VelocityProfileTiming::compute_v_m(double v_0, double v_f, double a_m, double j_m, double d) const {
  auto a = 1 / a_m;
  auto b = a_m / j_m;
  auto c = (std::pow(a_m, 2) * (v_0 + v_f) - j_m * (std::pow(v_0, 2) + std::pow(v_f, 2))) / (2 * a_m * j_m) - d;
  return (-b + sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
}

double VelocityProfileTiming::three_phase_profile_d(double v_a, double v_b) const {
  return (4 * std::pow(a_m, 2) + 6 * std::pow(a_m, 3) - 2 * std::pow(a_m, 4) + 4 * std::pow(a_m, 3) * j_m
      + std::pow(a_m, 2) * j_m * v_a + 2 * v_b * std::pow(a_m, 2) * j_m - std::pow(j_m, 2) * std::pow(v_a, 2))
      / (2 * a_m * std::pow(j_m, 2));
}

double VelocityProfileTiming::two_phase_stop_profile_d(double v_begin) const {
  return (v_begin + v_f) * (std::pow(a_m, 2) + j_m * v_begin - j_m * v_f) / (2 * a_m * j_m);

}

double VelocityProfileTiming::two_phase_start_profile_d(double v_end) const {
  return (v_0 + v_end) * (std::pow(a_m, 2) - j_m * v_0 + j_m * v_end) / (2 * a_m * j_m);

}

VelocityProfileTiming::VelocityProfileTiming(double d, double v_0, double v_f)
    : d(d),
      v_0(v_0),
      v_f(v_f),
      t_1(0),
      t_2(0),
      t_m1(0),
      t_m2(0),
      t_3(0),
      t_4(0),
      t_f(0),
      v_m(0),
      v_1(0),
      v_2(0),
      v_3(0),
      v_4(0) {
  double v_m_theoretical = compute_v_m(v_0, v_f, a_m, j_m, d);
  v_m = std::min(v_m_theoretical, MAX_SPEED_CUPS);

  if (v_f > v_0) {
    auto three_phase_start_profile_d = three_phase_profile_d(v_0, v_f);
    if (three_phase_start_profile_d > d) {
      v_m = v_f;
    }
  }

  t_1 = a_m / j_m;
  v_1 = v_0 + std::pow(a_m, 2) / (2 * j_m);
  v_2 = v_m - std::pow(a_m, 2) / (2 * j_m);
  t_2 = t_1 + (v_2 - v_1) / a_m;
  t_m1 = t_2 + t_1;

  if (v_m_theoretical > MAX_SPEED_CUPS) {
    t_m2 = t_m1 + (d - profile_distance(v_0, v_f, a_m, j_m, MAX_SPEED_CUPS)) / MAX_SPEED_CUPS;
  } else {
    t_m2 = t_m1;
  }

  if (v_f < v_0) {
    auto three_phase_stop_d = three_phase_profile_d(v_f, v_0);
    auto two_phase_stop_d = two_phase_stop_profile_d(v_0);
    if (three_phase_stop_d > d) {
      t_1 = 0;
      t_2 = 0;
      t_m1 = 0;
      t_m2 = (d - two_phase_stop_d) / v_0;
      v_m = v_0;
    }
  }

  // ramp down
  t_3 = t_m2 + a_m / j_m;
  v_3 = v_m - std::pow(a_m, 2) / (2 * j_m);
  v_4 = v_f + std::pow(a_m, 2) / (2 * j_m);
  t_4 = t_3 - (v_4 - v_3) / a_m;
  t_f = t_4 + a_m / j_m;

  // for when we don't have enough distance to slow down after speeding up
  if (v_f > v_0) {
    auto three_phase_start_d = three_phase_profile_d(v_0, v_f);
    auto two_phase_start_d = two_phase_start_profile_d(v_f);
    if (three_phase_start_d > d) {
      t_m2 = t_m1 + (d - two_phase_start_d) / v_f;
      v_m = v_f;
      t_3 = t_m2;
      v_3 = v_f;
      t_4 = t_m2;
      v_4 = v_f;
      t_f = t_m2;
    }
  }

//  print("%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\r\n", t_1, t_2, t_m1, t_m2, t_3, t_4, t_f);
//  print("%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\r\n", v_0, v_1, v_2, v_m, v_3, v_4, v_f);
//  print("%0.3f\r\n", d);
}

double VelocityProfileTiming::compute_forward_velocity(double t) const {

  double v_t = 0;
  if (t <= t_1) {
    v_t = v_0 + j_m * std::pow(t, 2) / 2.0;
  } else if (t <= t_2) {
    v_t = v_1 + a_m * (t - t_1);
  } else if (t <= t_m1) {
    v_t = v_2 + a_m * (t - t_2) - j_m * std::pow(t - t_2, 2) / 2.0;
  } else if (t <= t_m2) {
    v_t = v_m;
  } else if (t <= t_3) {
    v_t = v_m - j_m * std::pow(t - t_m2, 2) / 2.0;
  } else if (t <= t_4) {
    v_t = v_3 - a_m * (t - t_3);
  } else if (t <= t_f) {
    v_t = v_4 - a_m * (t - t_4) + j_m * std::pow(t - t_4, 2) / 2.0;
  } else {
    v_t = v_f;
  }

  return v_t;
}

}
}

