#pragma once

#include <utility>
#include <common/core/util.h>
#include <common/core/Pose.h>
#include <common/KinematicController/RobotConfig.h>
#include <common/KinematicController/TrajectoryPlanner.h>
#include <common/core/Mouse.h>
#include <common/KinematicController/RegulatedMotor.h>
#include <tuple>

namespace smartmouse {
namespace kc {

constexpr std::pair<double, double> from_sensors_to_wall(smartmouse::kc::SensorPose s1,
                                                         smartmouse::kc::SensorPose s2,
                                                         double s1_dist_m,
                                                         double s2_dist_m) {
  const double d1x = cos(s1.angle) * s1_dist_m + s1.x;
  const double d2x = cos(s2.angle) * s2_dist_m + s2.x;
  const double d1y = sin(s1.angle) * s1_dist_m + s1.y;
  const double d2y = sin(s2.angle) * s2_dist_m + s2.y;
  const double yaw = -atan2(d2y - d1y, d2x - d1x);
  // distance from point to line:
  // https://en.wikipedia.org/w/index.php?title=Distance_from_a_point_to_a_line&oldid=828284744
  // where x_0 and y_0 are equal to 0
  // the fabs is required because the numerator represents an area
  const double dist = fabs(d2x * d1y - d2y * d1x) / sqrt(pow(d2y - d1y, 2) + pow(d2x - d1x, 2));
  return {dist, yaw};
};

const std::pair<double, double> from_sensors_to_left_wall(smartmouse::kc::SensorPose s1,
                                                          smartmouse::kc::SensorPose s2,
                                                          double s1_dist_m,
                                                          double s2_dist_m);

const std::pair<double, double> from_sensors_to_right_wall(smartmouse::kc::SensorPose s1,
                                                           smartmouse::kc::SensorPose s2,
                                                           double s1_dist_m,
                                                           double s2_dist_m);

}
}

class KinematicController {
 public:
  KinematicController(Mouse *mouse);

  static double dispToNextEdge(Mouse &mouse);

  static double dispToNthEdge(Mouse &mouse, unsigned int n);

  static GlobalPose poseOfToNthEdge(Mouse &mouse, unsigned int n);

  static double fwdDispToCenter(Mouse &mouse);

  static double fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose);

  static GlobalPose forwardKinematics(double vl, double vr, double yaw, double dt);

  void planTraj(Waypoints waypoints);

  static double sidewaysDispToCenter(Mouse &mouse);

  std::tuple<double, double, bool> estimate_pose(RangeData range_data, Mouse &mouse);

  GlobalPose getGlobalPose();

  LocalPose getLocalPose();

  std::pair<double, double> getWheelVelocitiesCPS();

  bool isStopped();

  void reset_col_to(double new_col);

  void reset_row_to(double new_row);

  void reset_yaw_to(double new_yaw);

  std::pair<double, double> run(double dt_s, double left_angle_rad, double right_angle_rad, RangeData range_data);

  void setAccelerationCpss(double acceleration_mpss);

  void setSpeedCps(double left_setpoint_mps, double right_setpoint_mps);

  static const double kPWall;
  static const double kDWall;
  static const double kPYaw;

  void setParams(double kP, double kI, double kD, double ff_scale, double ff_offset);

  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

  bool enable_sensor_pose_estimate;
  bool enabled;
  bool kinematics_enabled;
  unsigned int row;
  unsigned int col;

  double getCurrentForwardSpeedCUPS();

 private:
  bool initialized;
  bool ignoring_left;
  bool ignoring_right;

  GlobalPose current_pose_estimate_cu;
  Mouse *mouse;
  double d_until_left_drop;
  double d_until_right_drop;
  static const double kDropSafety;
  double acceleration_cellpss;
  double dt_s;
};
