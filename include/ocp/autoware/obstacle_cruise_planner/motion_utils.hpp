#pragma once

#include <common/common_types.hpp>

#include <cstddef>
#include <algorithm>
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

inline double clamp01(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}

inline double calcSignedArcLengthToPoint(
  const std::vector<autoware::common_types::TrajectoryPoint> & points,
  const autoware::common_types::Point & point)
{
  if (points.size() < 2) {
    return 0.0;
  }
  double accumulated = 0.0;
  double best_dist = std::numeric_limits<double>::max();
  double best_s = 0.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & p0 = points.at(i).pose.position;
    const auto & p1 = points.at(i + 1).pose.position;
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double seg_len = std::hypot(vx, vy);
    if (seg_len <= 0.0) {
      continue;
    }
    const double wx = point.x - p0.x;
    const double wy = point.y - p0.y;
    double t = (wx * vx + wy * vy) / (seg_len * seg_len);
    t = clamp01(t);
    const double proj_x = p0.x + t * vx;
    const double proj_y = p0.y + t * vy;
    const double dist = std::hypot(point.x - proj_x, point.y - proj_y);
    const double s = accumulated + t * seg_len;
    if (dist < best_dist) {
      best_dist = dist;
      best_s = s;
    }
    accumulated += seg_len;
  }
  return best_s;
}

inline autoware::common_types::Orientation normalizeOrientation(
  const autoware::common_types::Orientation & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= 0.0) {
    return q;
  }
  autoware::common_types::Orientation out = q;
  out.x /= norm;
  out.y /= norm;
  out.z /= norm;
  out.w /= norm;
  return out;
}

inline autoware::common_types::Orientation slerpOrientation(
  autoware::common_types::Orientation q1, autoware::common_types::Orientation q2, double t)
{
  q1 = normalizeOrientation(q1);
  q2 = normalizeOrientation(q2);
  t = clamp01(t);
  double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
  if (dot < 0.0) {
    dot = -dot;
    q2.x = -q2.x;
    q2.y = -q2.y;
    q2.z = -q2.z;
    q2.w = -q2.w;
  }
  constexpr double kEps = 1e-6;
  if (1.0 - dot < kEps) {
    autoware::common_types::Orientation out;
    out.x = q1.x + t * (q2.x - q1.x);
    out.y = q1.y + t * (q2.y - q1.y);
    out.z = q1.z + t * (q2.z - q1.z);
    out.w = q1.w + t * (q2.w - q1.w);
    return normalizeOrientation(out);
  }
  const double theta = std::acos(dot);
  const double sin_theta = std::sin(theta);
  const double w1 = std::sin((1.0 - t) * theta) / sin_theta;
  const double w2 = std::sin(t * theta) / sin_theta;
  autoware::common_types::Orientation out;
  out.x = q1.x * w1 + q2.x * w2;
  out.y = q1.y * w1 + q2.y * w2;
  out.z = q1.z * w1 + q2.z * w2;
  out.w = q1.w * w1 + q2.w * w2;
  return normalizeOrientation(out);
}

inline autoware::common_types::TrajectoryPoint interpolateTrajectoryPoint(
  const autoware::common_types::TrajectoryPoint & front,
  const autoware::common_types::TrajectoryPoint & back, const double ratio)
{
  const double t = clamp01(ratio);
  autoware::common_types::TrajectoryPoint out = front;
  out.pose.position.x = front.pose.position.x + t * (back.pose.position.x - front.pose.position.x);
  out.pose.position.y = front.pose.position.y + t * (back.pose.position.y - front.pose.position.y);
  out.pose.position.z = front.pose.position.z + t * (back.pose.position.z - front.pose.position.z);
  out.pose.orientation = slerpOrientation(front.pose.orientation, back.pose.orientation, t);
  out.longitudinal_velocity_mps =
    front.longitudinal_velocity_mps + t * (back.longitudinal_velocity_mps - front.longitudinal_velocity_mps);
  out.lateral_velocity_mps =
    front.lateral_velocity_mps + t * (back.lateral_velocity_mps - front.lateral_velocity_mps);
  out.acceleration_mps2 = front.acceleration_mps2 + t * (back.acceleration_mps2 - front.acceleration_mps2);
  out.heading_rate_rps = front.heading_rate_rps + t * (back.heading_rate_rps - front.heading_rate_rps);
  out.front_wheel_angle_rad =
    front.front_wheel_angle_rad + t * (back.front_wheel_angle_rad - front.front_wheel_angle_rad);
  out.rear_wheel_angle_rad =
    front.rear_wheel_angle_rad + t * (back.rear_wheel_angle_rad - front.rear_wheel_angle_rad);
  out.time_from_start.sec =
    static_cast<std::int32_t>(front.time_from_start.sec +
                              t * (back.time_from_start.sec - front.time_from_start.sec));
  out.time_from_start.nanosec =
    static_cast<std::uint32_t>(front.time_from_start.nanosec +
                               t * (back.time_from_start.nanosec - front.time_from_start.nanosec));
  return out;
}

inline std::optional<size_t> insertTargetPoint(
  const size_t seg_idx, const autoware::common_types::Point & target,
  std::vector<autoware::common_types::TrajectoryPoint> & points, const double overlap_threshold = 1e-3)
{
  if (points.empty() || seg_idx + 1 >= points.size()) {
    return std::nullopt;
  }
  const auto & front = points.at(seg_idx);
  const auto & back = points.at(seg_idx + 1);
  const autoware::common_types::Pose front_pose = front.pose;
  const autoware::common_types::Pose back_pose = back.pose;
  const double segment_length = calcDistance(front_pose, back_pose);
  if (segment_length <= 0.0) {
    return std::nullopt;
  }
  const double dist_front = std::hypot(target.x - front_pose.position.x, target.y - front_pose.position.y);
  const double dist_back = std::hypot(target.x - back_pose.position.x, target.y - back_pose.position.y);
  if (dist_front < overlap_threshold) {
    return seg_idx;
  }
  if (dist_back < overlap_threshold) {
    return seg_idx + 1;
  }
  const double ratio = clamp01(dist_front / segment_length);
  auto inserted = interpolateTrajectoryPoint(front, back, ratio);
  inserted.pose.position.x = target.x;
  inserted.pose.position.y = target.y;
  inserted.pose.position.z = target.z;
  points.insert(points.begin() + static_cast<std::ptrdiff_t>(seg_idx + 1), inserted);
  return seg_idx + 1;
}

inline std::optional<size_t> insertTargetPoint(
  const double insert_length, const autoware::common_types::Point & target,
  std::vector<autoware::common_types::TrajectoryPoint> & points, const double overlap_threshold = 1e-3)
{
  if (points.empty() || insert_length < 0.0) {
    return std::nullopt;
  }
  double accumulated = 0.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const double segment = calcDistance(points.at(i).pose, points.at(i + 1).pose);
    if (accumulated + segment >= insert_length) {
      return insertTargetPoint(i, target, points, overlap_threshold);
    }
    accumulated += segment;
  }
  return std::nullopt;
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
  if (points.empty() || target_dist < 0.0) {
    return std::nullopt;
  }
  double accumulated = 0.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & front_pose = points.at(i).pose;
    const auto & back_pose = points.at(i + 1).pose;
    const double segment = calcDistance(front_pose, back_pose);
    if (accumulated + segment + 1e-3 > target_dist) {
      const double insert_length = target_dist - accumulated;
      const double ratio = segment > 0.0 ? clamp01(insert_length / segment) : 0.0;
      autoware::common_types::Point target;
      target.x = front_pose.position.x + ratio * (back_pose.position.x - front_pose.position.x);
      target.y = front_pose.position.y + ratio * (back_pose.position.y - front_pose.position.y);
      target.z = front_pose.position.z + ratio * (back_pose.position.z - front_pose.position.z);
      const auto stop_idx = insertTargetPoint(i, target, points, 1e-3);
      if (!stop_idx) {
        return std::nullopt;
      }
      for (size_t j = *stop_idx; j < points.size(); ++j) {
        points.at(j).longitudinal_velocity_mps = 0.0;
      }
      return stop_idx;
    }
    accumulated += segment;
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
