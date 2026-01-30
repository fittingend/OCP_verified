#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"

#include "autoware/obstacle_cruise_planner/motion_utils.hpp"

#include <utility>

namespace autoware::obstacle_cruise_planner::polygon_utils
{
namespace ct = autoware::common_types;

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points, const ct::StopObstacle & obstacle)
{
  if (traj_points.empty()) {
    return std::nullopt;
  }
  ct::Pose pose{};
  pose.position.x = obstacle.collision_point.x;
  pose.position.y = obstacle.collision_point.y;
  pose.position.z = obstacle.collision_point.z;
  const double distance = motion_utils::calcSignedArcLengthToPoint(traj_points, obstacle.collision_point);
  return std::make_pair(obstacle.collision_point, distance);
}

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points,
  const std::vector<ct::PredictedPath> & predicted_paths)
{
  if (traj_points.empty() || predicted_paths.empty() || predicted_paths.front().path.empty()) {
    return std::nullopt;
  }
  std::optional<std::pair<ct::Point, double>> best{};
  for (const auto & point : predicted_paths.front().path) {
    const double distance = motion_utils::calcSignedArcLengthToPoint(traj_points, point);
    if (!best || distance < best->second) {
      best = std::make_pair(point, distance);
    }
  }
  return best;
}

std::vector<ct::PointWithStamp> getCollisionPoints(const ct::PredictedObject & object)
{
  std::vector<ct::PointWithStamp> points;
  double stamp = 0.0;
  const auto & path = object.predicted_paths.empty()
                         ? std::vector<ct::Point>{}
                         : object.predicted_paths.front().path;
  for (const auto & point : path) {
    ct::PointWithStamp entry;
    entry.point = point;
    entry.stamp = stamp;
    stamp += object.predicted_paths.empty() ? 0.0 : object.predicted_paths.front().time_step;
    points.push_back(entry);
  }
  return points;
}
}  // namespace autoware::obstacle_cruise_planner::polygon_utils
