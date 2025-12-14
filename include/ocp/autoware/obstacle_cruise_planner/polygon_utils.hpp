#pragma once

#include <common/common_types.hpp>

#include <optional>
#include <vector>

namespace autoware::obstacle_cruise_planner::polygon_utils
{
namespace ct = autoware::common_types;

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points, const ct::StopObstacle & obstacle);

std::vector<ct::PointWithStamp> getCollisionPoints(const ct::PredictedObject & object);
}  // namespace autoware::obstacle_cruise_planner::polygon_utils
