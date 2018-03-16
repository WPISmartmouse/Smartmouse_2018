#include <algorithm>

#include <common/KinematicController/VelocityProfileTiming.h>
#include "RobotConfig.h"

namespace smartmouse {
namespace kc {

constexpr double profile_distance(double v_0, double v_f, double a_m, double j_m, double v_m) {
  return (std::pow(a_m, 2) * (v_0 + v_f + 2 * v_m) - j_m * (std::pow(v_0, 2) + std::pow(v_f, 2) - 2 * std::pow(v_m, 2)))
      / (2 * a_m * j_m);
}

constexpr double compute_v_m(double v_0, double v_f, double a_m, double j_m, double d) {
  auto a = 1 / a_m;
  auto b = a_m / j_m;
  auto c = (std::pow(a_m, 2) * (v_0 + v_f) - j_m * (std::pow(v_0, 2) + std::pow(v_f, 2))) / (2 * a_m * j_m) - d;
  return (-b + sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
}

VelocityProfileTiming::VelocityProfileTiming(double d, double v_0, double v_f)
    : d(d),
      v_0(v_0),
      v_f(v_f)
{
  double v_m_theoretical = compute_v_m(v_0, v_f, a_m, j_m, d);
  v_m = std::min(v_m_theoretical, MAX_SPEED_CUPS);
  t_1 = a_m / j_m;
  v_1 = v_0 + std::pow(a_m, 2) / (2 * j_m);
  v_2 = v_m - std::pow(a_m, 2) / (2 * j_m);
  t_2 = t_1 + (v_2 - v_1) / a_m;
  t_m1 = t_2 + t_1;

  if (v_m_theoretical > MAX_SPEED_CUPS) {
    t_m2  = t_m1 + (d - profile_distance(v_0, v_f, a_m, j_m, MAX_SPEED_CUPS)) / MAX_SPEED_CUPS;
  }
  else {
    t_m2 = t_m1;
  }
  t_3 = t_m2 + a_m / j_m;
  v_3 = v_m - std::pow(a_m, 2) / (2 * j_m);
  v_4 = v_f + std::pow(a_m, 2) / (2 * j_m);
  t_4 = t_3 - (v_4 - v_3) / a_m;
  t_f = t_4 + t_1;

//  print("%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", t_1, t_2, t_m1, t_m2, t_3, t_4, t_f);
//  print("%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", v_0, v_1, v_2, v_m, v_3, v_4, v_f);
//  print("%0.3f\n", d);
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

