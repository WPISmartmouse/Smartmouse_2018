#pragma once

#include <ignition/math.hh>
#include <experimental/optional>

class RayTracing {

 public:
  static std::experimental::optional<double> distance_to_wall(const ignition::math::Line2d wall,
                               const ignition::math::Vector2d pt,
                               ignition::math::Vector2d u);

};
