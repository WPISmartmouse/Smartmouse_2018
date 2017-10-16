#pragma once

#include <common/core/AbstractMaze.h>

namespace smartmouse {
namespace kc {

// most stuff here is meters or meters/second
constexpr double FRONT_ANALOG_ANGLE = 1.35255;
constexpr double BACK_ANALOG_ANGLE = 1.35255;
constexpr double FRONT_SIDE_ANALOG_X = 0.042;
constexpr double FRONT_SIDE_ANALOG_Y = 0.0350;
constexpr double BACK_SIDE_ANALOG_X = -0.0206;
constexpr double BACK_SIDE_ANALOG_Y = 0.0335;
constexpr double FRONT_ANALOG_X = 0.056;
constexpr double GERALD_X = 0.054;
constexpr double GERALD_Y = 0.018;
constexpr double GERALD_ANGLE = 0.6544985;
extern double MAX_SPEED;
constexpr double MAX_HARDWARE_SPEED = 0.60;
constexpr double MIN_SPEED = 0.014;
constexpr double FRONT_WALL_THRESHOLD = 0.15;
constexpr double SIDE_WALL_THRESHOLD = 0.08;
constexpr double GERALD_WALL_THRESHOLD = 0.17;
constexpr double WALL_CHANGED_THRESHOLD = 0.02;
constexpr double ROT_TOLERANCE = 0.05;
constexpr double TRACK_WIDTH = 0.0633;
constexpr double ANALOG_MAX_DIST = 0.18;
constexpr double WHEEL_RAD = 0.0145;
constexpr double MIN_ABSTRACT_FORCE = 3.5;
constexpr double END_SPEED= 0.3; // this can be lowered to 0.15 to demonstrate ForwardN
extern bool ARC_TURN;

constexpr double cellsToRad(double x) {
  return x * smartmouse::maze::UNIT_DIST_M / WHEEL_RAD;
}

constexpr double meterToRad(double x) {
  return x / WHEEL_RAD;
}

constexpr double radToMeters(double x) {
  return x * WHEEL_RAD;
}

}
}
