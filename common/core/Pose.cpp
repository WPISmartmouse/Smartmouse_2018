#include "Pose.h"

LocalPose::LocalPose() {}

LocalPose::LocalPose(double to_left, double to_back) : to_left(to_left), to_back(to_back) {}

LocalPose::LocalPose(double to_left, double to_back, double yaw_from_straight) : to_left(to_left), to_back(to_back),
                                                                                 yaw_from_straight(yaw_from_straight) {}

GlobalPose::GlobalPose() : col(0), row(0), yaw(0) {}

GlobalPose::GlobalPose(double col, double row) : col(col), row(row), yaw(0) {}

GlobalPose::GlobalPose(double col, double row, double yaw) : col(col), row(row), yaw(yaw) {}

GlobalState::GlobalState() {}

GlobalState::GlobalState(GlobalPose pose, double velocity) : pose(pose), velocity(velocity) {}
