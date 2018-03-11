#include <common/KinematicController/KinematicController.h>
#include <common/KinematicController/DriveStraightState.h>
#include <common/math/math.h>

namespace smartmouse {
namespace kc {

const double kVf = 1.0;

DriveStraightState::DriveStraightState(GlobalPose start_pose, double goalDisp, double v_initial, double v_final)
    : goal_disp(goalDisp),
      v_0(v_initial),
      v_f(v_final),
      t_1(a_m / j_m),
      v_1(v_0 + std::pow(a_m, 2) / (2 * j_m)),
      v_2(v_f - std::pow(a_m, 2) / (2 * j_m)),
      t_2(t_1 + (v_2 - v_1) / a_m),
      t_f(t_2 + t_1),
      disp(0),
      start_pose(start_pose) {
}

std::pair<double, double> DriveStraightState::compute_wheel_velocities(Mouse &mouse, double t_s) {
  GlobalPose current_pose = mouse.getGlobalPose();
  disp = KinematicController::fwdDisp(mouse.getDir(), current_pose, start_pose);

  const double error_to_center_cu = KinematicController::sidewaysDispToCenter(mouse);
  const double goal_yaw = dir_to_yaw(mouse.getDir()) + error_to_center_cu * kPYaw;

  // The goal is to be facing straight when you wall distance is correct.
  // To achieve this, we control our yaw as a function of our error in wall distance
  const double yaw_error = smartmouse::math::yaw_diff(goal_yaw, current_pose.yaw);

  // given starting velocity, max velocity, acceleration, and jerk, and final velocity...
  // generate the velocity profile for achieving this as fast as possible
  double forward_velocity = compute_forward_velocity(t_s); // FIXME: our goal is to calculate this!

  double correction = kPWall * yaw_error;
  if (yaw_error < 0) { // need to turn clockwise
    return {forward_velocity, forward_velocity - correction};
  } else {
    // correction will be negative here, so add it
    return {forward_velocity + correction, forward_velocity};
  }
}

double DriveStraightState::dispError() {
  return goal_disp - disp;
}

double DriveStraightState::compute_forward_velocity(double t /* seconds */ ) {
  // see the docs/Time Optimal Smartmouse Controls notebook for documentation
  double v_t = 0;
  if (t <= t_1) {
    v_t = v_0 + j_m * std::pow(t, 2) / 2.0;
  } else if (t <= t_2) {
    v_t = v_1 + a_m * (t - t_1);
  } else if (t <= t_f) {
    v_t = v_2 + a_m * (t - t_2) - j_m * std::pow(t - t_2, 2) / 2.0;
  } else {
    v_t = v_f;
  }

  return v_t;
}

}
}
