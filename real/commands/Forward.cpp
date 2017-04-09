#include <real/RealMouse.h>
#include <tuple>
#include "Forward.h"

Forward::Forward() : Command("Forward"), mouse(RealMouse::inst()), follower(RealMouse::CONFIG) {}


void Forward::initialize() {
  start = mouse->getPose();
  follower.goalDisp = WallFollower::dispToNextEdge(mouse);
  print("forward\n");
}

void Forward::execute() {
  range_data = mouse->getRangeData();

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse, start, range_data);
  mouse->setSpeed(l, r);

//  const double RST = 0.15;
//  if (range_data == RST) {
//    switch (mouse->getDir()) {
//      case Direction::N: {
//        double next_row_y = (mouse->getRow() - 1) * AbstractMaze::UNIT_DIST;
//        mouse->kinematic_controller.reset_y_to(RealMouse::CONFIG.FRONT_BINARY_THRESHOLD + next_row_y + RealMouse::CONFIG.FRONT_BINARY_X);
//        break;
//      }
//      case Direction::S: {
//        double next_row_y = (mouse->getRow() + 2) * AbstractMaze::UNIT_DIST;
//        mouse->kinematic_controller.reset_y_to(next_row_y - RealMouse::CONFIG.FRONT_BINARY_THRESHOLD - RealMouse::CONFIG.FRONT_BINARY_X);
//        break;
//      }
//      case Direction::E: {
//        double next_col_x = (mouse->getCol() + 2) * AbstractMaze::UNIT_DIST;
//        mouse->kinematic_controller.reset_x_to(next_col_x - RealMouse::CONFIG.FRONT_BINARY_THRESHOLD - RealMouse::CONFIG.FRONT_BINARY_X);
//        break;
//      }
//      case Direction::W: {
//        double next_col_x = (mouse->getCol() - 1) * AbstractMaze::UNIT_DIST;
//        mouse->kinematic_controller.reset_x_to(RealMouse::CONFIG.FRONT_BINARY_THRESHOLD + next_col_x  + RealMouse::CONFIG.FRONT_BINARY_X);
//        break;
//      }
//    }
//    mouse->reset_fwd_dist = false;
//  }
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
}

