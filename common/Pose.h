#pragma once

class Pose {
public:
  Pose();

  Pose(double x, double y);

  Pose(double x, double y, double yaw);

  double x, y;
  double yaw;
};
