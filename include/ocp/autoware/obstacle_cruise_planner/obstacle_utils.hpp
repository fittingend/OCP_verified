#pragma once

#include <common/common_types.hpp>

#include <algorithm>
#include <vector>
#include <limits>

namespace autoware::obstacle_cruise_planner::obstacle_utils
{
namespace ct = autoware::common_types;

inline std::vector<ct::StopObstacle> getClosestStopObstacles(
  const std::vector<ct::StopObstacle> & stop_obstacles, const double max_distance = std::numeric_limits<double>::infinity())
{
  auto sorted = stop_obstacles;
  if (!std::isinf(max_distance)) {
    for (auto & stop_obstacle : sorted) {
      stop_obstacle.dist_to_collide_on_decimated_traj =
        std::min(stop_obstacle.dist_to_collide_on_decimated_traj, max_distance);
    }
  }
  std::sort(
    sorted.begin(), sorted.end(),
    [](const ct::StopObstacle & lhs, const ct::StopObstacle & rhs) {
      return lhs.dist_to_collide_on_decimated_traj < rhs.dist_to_collide_on_decimated_traj;
    });
  return sorted;
}
}  // namespace autoware::obstacle_cruise_planner::obstacle_utils
