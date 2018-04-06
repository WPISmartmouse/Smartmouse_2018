#include <tuple>
#include <common/math/math.h>
#include <real/commands/TurnInPlace.h>

const double TurnInPlace::kP = 1.0;

TurnInPlace::TurnInPlace(Direction dir)
    : Command("RealTurnInPlace"), mouse(RealMouse::inst()), left_turn(false), dir(dir), goal_yaw(0), yaw_error(0) {}

void TurnInPlace::initialize() {
  mouse->kinematic_controller.enable_sensor_pose_estimate = false;
  digitalWrite(RealMouse::LED_2, 1);
  setTimeout(5000);
  goal_yaw = dir_to_yaw(dir);
  const auto start = mouse->getGlobalPose();
  yaw_error = smartmouse::math::yaw_diff(start.yaw, goal_yaw);
  const double turn_arc_length = fabs(yaw_error) * smartmouse::kc::TRACK_WIDTH_CU / 2;
  left_turn = (yaw_error < 0);
  profile = new smartmouse::kc::VelocityProfile(start, {turn_arc_length, 0, 0});
}

void TurnInPlace::execute() {
//  print("turning %f\r\n", yaw_error);
  double t_s = getTime() / 1000.0;
  double fwd_v = profile->compute_forward_velocity(t_s);
  double vl = 0, vr = 0;
  if (left_turn) {
    vl = -fwd_v;
    vr = fwd_v;
  } else {
    vl = fwd_v;
    vr = -fwd_v;
  }

  if (vl < .01 and vr < .01 and fabs(yaw_error) > smartmouse::kc::ROT_TOLERANCE) {
    vl += kP * yaw_error;
    vr -= kP * yaw_error;
  }

  mouse->setSpeedCps(vl, vr);

  // when we get close to aligned, there might be a wall we can use to better estimate our angle
  // this allows us to use that
  if (fabs(yaw_error) < smartmouse::kc::ROT_TOLERANCE * 4 && mouse->kinematic_controller.enable_sensor_pose_estimate) {
    mouse->kinematic_controller.enable_sensor_pose_estimate = true;
    // FIXME: this is kind of a hack. It's needed because DriveStraight checks dir in order to compute
    // FIXME: the correct yaw. it adds dir_to_yaw(getDir()), so we must assume we're close enough
    mouse->internalTurnToFace(dir);
  }
}

bool TurnInPlace::isFinished() {
  double current_yaw = mouse->getGlobalPose().yaw;
  yaw_error = smartmouse::math::yaw_diff(current_yaw, goal_yaw);
  double vl_cps, vr_cps;
  std::tie(vl_cps, vr_cps) = mouse->getWheelVelocitiesCPS();
  return isTimedOut()
      || ((fabs(yaw_error) < smartmouse::kc::ROT_TOLERANCE) && fabs(vl_cps) <= smartmouse::kc::MIN_SPEED_CUPS
          && fabs(vr_cps) < smartmouse::kc::MIN_SPEED_CUPS);
}

void TurnInPlace::end() {
  mouse->kinematic_controller.enable_sensor_pose_estimate = true;
  digitalWrite(RealMouse::LED_2, 0);
  mouse->internalTurnToFace(dir);
}

