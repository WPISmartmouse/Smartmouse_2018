#ifdef SIM

#include <SimMouse.h>
#include "Forward.h"

Forward::Forward() : mouse(SimMouse::inst()), checkedWalls(false), wallOnLeft(true), wallOnRight(true) {}


void Forward::initialize() {
//  mouse->resetIndicators(SimMouse::red_color);
//  mouse->indicatePath(mouse->getRow(), mouse->getCol(), mouse->maze->pathToNextGoal, SimMouse::red_color);
  start = mouse->getEstimatedPose();
  follower.goalDisp = WallFollower::dispToEdge(mouse);
}

void Forward::execute() {
  range_data = mouse->getRangeData();

//  if (range_data.front_binary) {
//    disp = AbstractMaze::UNIT_DIST - SimMouse::FRONT_BINARY_THRESHOLD;
//  }
//  else {
//    disp = forwardDisplacement(start, mouse->getExactPose());
//  }

  double l, r;
  std::tie(l, r) = follower.compute_wheel_velocities(this->mouse, start, range_data);

  mouse->setSpeed(l, r);

  if (!checkedWalls && follower.dispError < AbstractMaze::HALF_UNIT_DIST) {
    checkedWalls = true;
    walls[static_cast<int>(right_of_dir(mouse->getDir()))] =
            follower.dToWallRight < SimMouse::WALL_DIST;
    walls[static_cast<int>(left_of_dir(mouse->getDir()))] =
            follower.dToWallLeft < SimMouse::WALL_DIST;
  }
}

bool Forward::isFinished() {
  return follower.dispError <= 0;
}

void Forward::end() {
//  mouse->resetIndicators(SimMouse::grey_color);
//  mouse->indicatePath(0, 0, mouse->maze->fastest_theoretical_route, SimMouse::blue_color);

  walls[static_cast<int>(mouse->getDir())] = range_data.front_binary;
  walls[static_cast<int>(opposite_direction(mouse->getDir()))] = false;

  mouse->suggestWalls(walls);
}

#endif
