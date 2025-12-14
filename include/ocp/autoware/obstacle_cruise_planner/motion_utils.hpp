#pragma once

#include <common/common_types.hpp>

#include <cstddef>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::obstacle_cruise_planner::motion_utils
{
inline double calcDistance(const autoware::common_types::Pose & lhs, const autoware::common_types::Pose & rhs)
{
  const double dx = lhs.position.x - rhs.position.x;
  const double dy = lhs.position.y - rhs.position.y;
  return std::hypot(dx, dy);
}

inline double calcSignedArcLength(
  const std::vector<autoware::common_types::TrajectoryPoint> & points, size_t start_idx, size_t end_idx)
{
  if (points.empty() || start_idx >= points.size() || end_idx >= points.size() ||
      start_idx > end_idx) {
    return 0.0;
  }
  double length = 0.0;
  for (size_t i = start_idx + 1; i <= end_idx; ++i) {
    const auto & prev_pose = points.at(i - 1).pose;
    const auto & curr_pose = points.at(i).pose;
    length += calcDistance(prev_pose, curr_pose);
  }
  return length;
}

inline size_t findNearestSegmentIndex(
  const std::vector<autoware::common_types::TrajectoryPoint> & points, const autoware::common_types::Pose & pose)
{
  if (points.empty()) {
    return 0;
  }
  size_t best = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < points.size(); ++i) {
    const double dist = calcDistance(points.at(i).pose, pose);
    if (dist < min_dist) {
      min_dist = dist;
      best = i;
    }
  }
  return best;
}

inline double calcSignedArcLengthToPose(
  const std::vector<autoware::common_types::TrajectoryPoint> & points,
  const autoware::common_types::Pose & pose)
{
  if (points.empty()) {
    return 0.0;
  }
  const size_t nearest_idx = findNearestSegmentIndex(points, pose);
  return calcSignedArcLength(points, 0, nearest_idx);
}

inline std::optional<size_t> insertStopPoint(
  const double /*start_dist*/, const double target_dist,
  std::vector<autoware::common_types::TrajectoryPoint> & points)
{
  if (points.empty()) {
    return std::nullopt;
  }
  double accumulated = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev_pose = points.at(i - 1).pose;
    const auto & curr_pose = points.at(i).pose;
    const double segment = calcDistance(prev_pose, curr_pose);
    accumulated += segment;
    if (accumulated >= target_dist || i == points.size() - 1) {
      points.at(i).longitudinal_velocity_mps = 0.0;
      for (size_t j = i + 1; j < points.size(); ++j) {
        points.at(j).longitudinal_velocity_mps = 0.0;
      }
      return i;
    }
  }
  return std::nullopt;
}

inline std::optional<size_t> getDistanceIndex(
  const std::vector<autoware::common_types::TrajectoryPoint> & points, const double distance)
{
  if (points.empty()) {
    return std::nullopt;
  }
  if (distance <= 0.0) {
    return 0;
  }
  double accumulated = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double segment = calcDistance(points.at(i - 1).pose, points.at(i).pose);
    accumulated += segment;
    if (accumulated >= distance) {
      return i;
    }
  }
  return points.size() - 1;
}
}  // namespace autoware::obstacle_cruise_planner::motion_utils
