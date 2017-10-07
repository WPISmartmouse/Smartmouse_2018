#pragma once

#include <ignition/math.hh>

class RayTracing {

 public:
  static bool distance_to_wall(const ignition::math::Line2d wall,
                               const ignition::math::Vector2d pt,
                               ignition::math::Vector2d u,
                               double *dist);

};
