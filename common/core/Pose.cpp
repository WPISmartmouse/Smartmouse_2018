#include "Pose.h"

LocalPose::LocalPose() {}

LocalPose::LocalPose(double to_left, double to_back) : to_left(to_left), to_back(to_back) {}

LocalPose::LocalPose(double to_left, double to_back, double yaw_from_straight) : to_left(to_left), to_back(to_back),
                                                                                 yaw_from_straight(yaw_from_straight) {}

GlobalPose::GlobalPose() : x(0), y(0), yaw(0) {}

GlobalPose::GlobalPose(double x, double y) : x(x), y(y), yaw(0) {}

GlobalPose::GlobalPose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
