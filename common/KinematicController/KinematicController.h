#pragma once

#include <utility>
#include <common/core/util.h>
#include <common/core/Pose.h>
#include <common/KinematicController/RobotConfig.h>
#include <common/KinematicController/TrajectoryPlanner.h>
#include <common/core/Mouse.h>
#include <common/KinematicController/RegulatedMotor.h>
#include <tuple>

struct drive_straight_state_t {
  double disp;
  double goalDisp;
  double dispError;
  GlobalPose start_pose;
  double start_time_s;
  double left_speed_mps;
  double right_speed_mps;
  double forward_v;
  double v_final;
};

class KinematicController {
public:
  KinematicController(Mouse *mouse);

  static double dispToNextEdge(Mouse *mouse);

  static double dispToNthEdge(Mouse *mouse, unsigned int n);

  static GlobalPose poseOfToNthEdge(Mouse *mouse, unsigned int n);

  static double fwdDispToCenter(Mouse *mouse);

  static double fwdDispToDiag(Mouse *mouse);

  static double fwdDisp(Direction dir, GlobalPose current_pose, GlobalPose start_pose);

  static double yawDiff(double y1, double y2);

  void start(GlobalPose start_pose, double goalDisp, double v_final=config.END_SPEED);

  void planTraj(Waypoints waypoints);

  double sidewaysDispToCenter(Mouse *mouse);

  std::pair<double, double> compute_wheel_velocities(Mouse *mouse);

  std::tuple<double, double, bool> estimate_pose(RangeData range_data, Mouse *mouse);

  GlobalPose getGlobalPose();

  LocalPose getLocalPose();

  std::pair<double, double> getWheelVelocities();

  bool isStopped();

  void reset_x_to(double new_x);

  void reset_y_to(double new_y);

  void reset_yaw_to(double new_yaw);

  std::pair<double, double> run(double dt_s, double left_angle_rad, double right_angle_rad, RangeData range_data);

  void setAccelerationMpss(double acceleration_mpss);

  void setSpeedMps(double left_setpoint_mps, double right_setpoint_mps);

  static const double kPWall;
  static const double kDWall;
  static const double kPYaw;

  drive_straight_state_t drive_straight_state;
  void setParams(double kP, double kI, double kD, double ff_offset, double int_cap);

  RegulatedMotor left_motor;
  RegulatedMotor right_motor;

  bool enable_sensor_pose_estimate;
  bool enabled;
  double row_offset_to_edge;
  double col_offset_to_edge;
  unsigned int row;
  unsigned int col;

private:
  bool initialized;
  bool ignoring_left;
  bool ignoring_right;

  GlobalPose current_pose_estimate;
  Mouse *mouse;
  double d_until_left_drop;
  double d_until_right_drop;
  static const double DROP_SAFETY;
  static const double POST_DROP_DIST;
  double acceleration_mpss;
  double dt_s;
};
