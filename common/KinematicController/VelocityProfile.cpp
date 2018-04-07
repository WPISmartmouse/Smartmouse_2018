#include <common/KinematicController/KinematicController.h>
#include <common/KinematicController/VelocityProfile.h>
#include <common/math/math.h>
#include "VelocityProfileTiming.h"

namespace smartmouse {
namespace kc {


VelocityProfile::VelocityProfile(GlobalPose start_pose, const VelocityProfileTiming timing)
    : timing(timing),
      disp(0),
      start_pose(start_pose) {
}

std::pair<double, double> VelocityProfile::drive_straight_wheel_velocities(Mouse &mouse, double t_s) {
  GlobalPose current_pose = mouse.getGlobalPose();
  disp = KinematicController::fwdDisp(mouse.getDir(), current_pose, start_pose);

  const double error_to_center_cu = KinematicController::sidewaysDispToCenter(mouse);
  const double goal_yaw = dir_to_yaw(mouse.getDir()) + error_to_center_cu * kPYaw;

  // The goal is to be facing straight when you wall distance is correct.
  // To achieve this, we control our yaw as a function of our error in wall distance
  const double yaw_error = smartmouse::math::yaw_diff(goal_yaw, current_pose.yaw);

  // given starting velocity, max velocity, acceleration, and jerk, and final velocity...
  // generate the velocity profile for achieving this as fast as possible
  double forward_velocity = compute_forward_velocity(t_s);

  double correction = kPWall * fabs(yaw_error);

  if (yaw_error < 0) {
    // need to turn clockwise
    return {forward_velocity, forward_velocity - correction};
  } else {
    // need to turn counter-clockwise
    return {forward_velocity - correction, forward_velocity};
  }
}

double VelocityProfile::dispError() {
  return timing.d - disp;
}

double VelocityProfile::compute_forward_velocity(double t /* seconds */ ) {
  // see the docs/Time Optimal Smartmouse Controls notebook for documentation
  return timing.compute_forward_velocity(t);
}

}
}
