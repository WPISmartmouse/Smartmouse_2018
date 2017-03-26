#include "Pose.h"

Pose::Pose() : x(0), y(0), yaw(0) {}

Pose::Pose(float x, float y) : x(x), y(y), yaw(0) {}

Pose::Pose(float x, float y, float yaw) : x(x), y(y), yaw(yaw) {}
