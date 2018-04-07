#pragma once

#include <common/core/AbstractMaze.h>
#include <cmath>
#include <common/IRSensorModeling/Model.h>

namespace smartmouse {
namespace kc {

struct SensorPose {
  double x;
  double y;
  double angle;
};

// most stuff here is meters or meters/second
constexpr double FRONT_ANALOG_ANGLE = 1.2217;
constexpr double BACK_ANALOG_ANGLE = 1.48353;
constexpr double FRONT_SIDE_ANALOG_X = 0.033;
constexpr double FRONT_SIDE_ANALOG_Y = 0.026;
constexpr double BACK_SIDE_ANALOG_X = -0.026;
constexpr double BACK_SIDE_ANALOG_Y = 0.03;
constexpr double FRONT_ANALOG_X = 0.045;
constexpr double GERALD_X = 0.040;
constexpr double GERALD_Y = 0.022;
constexpr double GERALD_ANGLE = 1.0472;
constexpr double MAX_HARDWARE_SPEED_MPS = 0.90;
constexpr double MIN_SPEED_MPS = 0.0145;
constexpr double FRONT_WALL_THRESHOLD = 0.12;
constexpr double SIDE_WALL_THRESHOLD = 0.12;
constexpr double GERALD_WALL_THRESHOLD = 0.15;
constexpr double WALL_CHANGED_THRESHOLD = 0.02;
constexpr double USE_FRONT_WALL_FOR_POSE = 0.08;
constexpr double ROT_TOLERANCE = 0.025;
constexpr double TRACK_WIDTH_M = 0.053;
constexpr double ANALOG_MAX_DIST_M = 0.16;
constexpr double ANALOG_MIN_DIST_M = 0.015;
constexpr double WHEEL_RAD = 0.0145;
constexpr double MIN_ABSTRACT_FORCE = 12;
constexpr int TICKS_PER_REVOLUTION = static_cast<const int>(std::pow(2, 14));
constexpr double RAD_PER_TICK = 2 * M_PI / TICKS_PER_REVOLUTION;

extern double MAX_SPEED_MPS;
extern bool ARC_TURN;
extern double MAX_SPEED_CUPS;

constexpr double TRACK_WIDTH_CU = smartmouse::maze::toCellUnits(TRACK_WIDTH_M);
constexpr double MAX_HARDWARE_SPEED_CUPS = smartmouse::maze::toCellUnits(MAX_HARDWARE_SPEED_MPS);
constexpr double MIN_SPEED_CUPS = smartmouse::maze::toCellUnits(MIN_SPEED_MPS);
constexpr double ANALOG_MAX_DIST_CU = smartmouse::maze::toCellUnits(ANALOG_MAX_DIST_M);
constexpr double ANALOG_MIN_DIST_CU = smartmouse::maze::toCellUnits(ANALOG_MIN_DIST_M);

constexpr SensorPose BACK_LEFT{BACK_SIDE_ANALOG_X, -BACK_SIDE_ANALOG_Y, -BACK_ANALOG_ANGLE};
constexpr SensorPose FRONT_LEFT{FRONT_SIDE_ANALOG_X, -FRONT_SIDE_ANALOG_Y, -FRONT_ANALOG_ANGLE};
constexpr SensorPose GERALD_LEFT{GERALD_X, -GERALD_Y, -GERALD_ANGLE};
constexpr SensorPose BACK_RIGHT{BACK_SIDE_ANALOG_X, BACK_SIDE_ANALOG_Y, BACK_ANALOG_ANGLE};
constexpr SensorPose FRONT_RIGHT{FRONT_SIDE_ANALOG_X, FRONT_SIDE_ANALOG_Y, FRONT_ANALOG_ANGLE};
constexpr SensorPose GERALD_RIGHT{GERALD_X, GERALD_Y, GERALD_ANGLE};

constexpr double cellsToRad(double x) {
  return x * smartmouse::maze::UNIT_DIST_M / WHEEL_RAD;
}

constexpr double meterToRad(double x) {
  return x / WHEEL_RAD;
}

constexpr double radToMeters(double x) {
  return x * WHEEL_RAD;
}

constexpr double radToCU(double x) {
  return x * WHEEL_RAD / smartmouse::maze::UNIT_DIST_M;
}

}
}
