#include <lib/common/RayTracing.h>
#include <common/core/AbstractMaze.h>

std::experimental::optional<double> RayTracing::distance_to_wall(const ignition::math::Line2d wall,
                                  const ignition::math::Vector2d pt,
                                  ignition::math::Vector2d u) {
  // project along u the size of the maze
  ignition::math::Vector2d u_proj(u);
  u_proj *= AbstractMaze::MAZE_SIZE_M;

  ignition::math::Vector2d intersection_point;
  bool intersects = wall.Intersect(ignition::math::Line2d(pt, pt + u_proj), intersection_point);

  if (intersects) {
    double dist = pt.Distance(intersection_point);
    return std::experimental::optional<double>(dist);
  }
  else {
    return std::experimental::optional<double>();
  }
}
