#pragma once

class LocalPose {
public:
  LocalPose();

  LocalPose(double to_left, double to_back);

  LocalPose(double to_left, double to_back, double yaw_from_straight);

  double to_left, to_back;
  double yaw_from_straight;
};

class GlobalPose {
public:
  GlobalPose();

  GlobalPose(double col, double row);

  GlobalPose(double col, double row, double yaw);

  double col, row;
  double yaw;
};

class GlobalState {
 public:
  GlobalState();

  GlobalState(GlobalPose pose, double velocity);

  GlobalPose pose;
  double velocity;
};
