#ifdef SIM

#include "Turn.h"

Turn::Turn(Direction dir) : mouse(SimMouse::inst()), dir(dir),
                            l(0), r(0), full_180(false), state(Turn::STOP) {}

void Turn::initialize() {
  start = mouse->getExactPose();
  double current_yaw = start.Rot().Yaw();
  goalYaw = toYaw(dir);

  // if it's close enough to 180, we do full stop then turn.
  if (fabs(yawDiff(current_yaw, goalYaw)) > (M_PI - 0.5)) {
    full_180 = true;
  }
}

double Turn::yawDiff(double y1, double y2) {
  double diff = y2 - y1;
  if (diff > M_PI) {
    return diff - 2 * M_PI;
  }
  if (diff < -M_PI) {
    return diff + 2 * M_PI;
  }
  return diff;
}

void Turn::execute() {
  double l, r;
  if (full_180) {
    switch (state) {
      case STOP:
        if (getTime() < 500) {
          mouse->setSpeed(0.02, 0.02);
        }
        else if (getTime() < 600) {
          mouse->setSpeed(0.0, 0.0);
        }
        else {
          state = TURN;
        }
        break;
      case TURN:
        l = -dYaw * kP;
        r = dYaw * kP;
        setSpeedLimited(l, r);
        if (fabs(dYaw) < Mouse::ROT_TOLERANCE) {
          mouse->internalTurnToFace(dir);
          state = GO_TO_EDGE;
          auto current_pose = mouse->getExactPose();
          switch (dir) {
            case Direction::S:
              follower.goalDisp = current_pose.Pos().Y() - start.Pos().Y();
              break;
            case Direction::N:
              follower.goalDisp = start.Pos().Y() - current_pose.Pos().Y();
              break;
            case Direction::W:
              follower.goalDisp = current_pose.Pos().X() - start.Pos().X();
              break;
            case Direction::E:
              follower.goalDisp = start.Pos().X() - current_pose.Pos().X();
              break;
          }
        }
        break;
      case GO_TO_EDGE:
        std::tie(l, r) = follower.compute_wheel_velocities(mouse, start, mouse->getRangeData());
        printf("%f, %f\n", l, r);
        mouse->setSpeed(0.02, 0.02);
        if (follower.dispError <= 0) {
          state = DONE;
        }
        break;
    }
  } else {
    l = -dYaw * kP;
    r = dYaw * kP;
    setSpeedLimited(l, r);

  }
}

void Turn::setSpeedLimited(double l, double r) {
  if (l > 0) {
    l = std::fmin(l, SimMouse::MAX_SPEED);
    l = std::fmax(l, SimMouse::MIN_SPEED);
  } else {
    l = std::fmax(l, -SimMouse::MAX_SPEED);
    l = std::fmin(l, -SimMouse::MIN_SPEED);
  }

  if (r > 0) {
    r = std::fmin(r, SimMouse::MAX_SPEED);
    r = std::fmax(r, SimMouse::MIN_SPEED);
  } else {
    r = std::fmax(r, -SimMouse::MAX_SPEED);
    r = std::fmin(r, -SimMouse::MIN_SPEED);
  }

  mouse->setSpeed(l, r);
}

bool Turn::isFinished() {
  double currentYaw = mouse->getExactPose().Rot().Yaw();
  dYaw = yawDiff(currentYaw, goalYaw);

  if (full_180) {
    return state == DONE && fabs(dYaw) < Mouse::ROT_TOLERANCE;
  } else {
    return fabs(dYaw) < Mouse::ROT_TOLERANCE;
  }
}

void Turn::end() {
  mouse->internalTurnToFace(dir);
}

#endif
