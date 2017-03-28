#pragma once

class Pose {
public:
  Pose();

  Pose(float x, float y);

  Pose(float x, float y, float yaw);

  float x, y;
  float yaw;
};
