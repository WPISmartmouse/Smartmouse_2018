#include <lib/common/RayTracing.h>
#include <common/core/AbstractMaze.h>

std::experimental::optional<double> RayTracing::distance_to_wall(const ignition::math::Line2d wall,
                                  const ignition::math::Vector2d pt,
                                  ignition::math::Vector2d u) {
  // project along u the size of the maze
  ignition::math::Vector2d u_proj(u);
  u_proj *= smartmouse::maze::SIZE_M;

  ignition::math::Vector2d intersection_point;
  ignition::math::Line2d sensor_ray(pt, pt + u_proj);
  bool intersects = wall.Intersect(sensor_ray, intersection_point);

  if (intersects) {
    double dist = pt.Distance(intersection_point);
    return std::experimental::optional<double>(dist);
  }
  else {
    return std::experimental::optional<double>();
  }
}
