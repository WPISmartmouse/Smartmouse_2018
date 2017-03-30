#include "Pose.h"

Pose::Pose() : x(0), y(0), yaw(0) {}

Pose::Pose(double x, double y) : x(x), y(y), yaw(0) {}

Pose::Pose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
