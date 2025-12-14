#include "autoware/obstacle_cruise_planner/planner_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace autoware::obstacle_cruise_planner
{
namespace ct = autoware::common_types;
PlannerInterface::PlannerInterface(const LongitudinalInfo & longitudinal_info)
: longitudinal_info_(longitudinal_info)
{
}

std::vector<autoware::common_types::TrajectoryPoint> PlannerInterface::generateStopTrajectory(
  const PlannerData & planner_data, const std::vector<autoware::common_types::StopObstacle> & stop_obstacles)
{
  auto shared_stop_obstacles = stop_obstacles;
  if (shared_stop_obstacles.empty()) {
    if (!planner_data.traj_points.empty()) {
      autoware::common_types::StopObstacle fallback;
      fallback.uuid = "goal_stop";
      const size_t last_idx = planner_data.traj_points.size() - 1;
      fallback.dist_to_collide_on_decimated_traj =
        motion_utils::calcSignedArcLength(planner_data.traj_points, 0, last_idx);
      const auto & final_pose = planner_data.traj_points.at(last_idx).pose;
      fallback.collision_point = ct::Point{
        final_pose.position.x, final_pose.position.y, final_pose.position.z};
      fallback.velocity = 0.0;
      fallback.stamp = {};
      shared_stop_obstacles.push_back(fallback);
    }
  }
  if (shared_stop_obstacles.empty()) {
    return planner_data.traj_points;
  }

  const double path_length = planner_data.traj_points.empty()
    ? 0.0
    : motion_utils::calcSignedArcLength(
        planner_data.traj_points, 0, planner_data.traj_points.size() - 1);
  const auto closest_stop_obstacles =
    obstacle_utils::getClosestStopObstacles(shared_stop_obstacles, path_length);

  std::optional<double> determined_zero_vel_dist{};
  for (const auto & stop_obstacle : closest_stop_obstacles) {
    const double desired_margin = calculateMarginFromObstacleOnCurve(planner_data, stop_obstacle);
    const double candidate_zero_vel_dist =
      std::max(0.0, stop_obstacle.dist_to_collide_on_decimated_traj - desired_margin);
    if (!determined_zero_vel_dist || candidate_zero_vel_dist < *determined_zero_vel_dist) {
      determined_zero_vel_dist = candidate_zero_vel_dist;
    }
  }

  if (!determined_zero_vel_dist) {
    return planner_data.traj_points;
  }

  auto output = planner_data.traj_points;
  motion_utils::insertStopPoint(0.0, *determined_zero_vel_dist, output);
  return output;
}

std::vector<autoware::common_types::TrajectoryPoint> PlannerInterface::generateSlowDownTrajectory(
    const PlannerData & planner_data,
    const std::vector<autoware::common_types::TrajectoryPoint> & cruise_traj_points,
    const std::vector<autoware::common_types::SlowDownObstacle> & obstacles,
  std::optional<autoware::common_types::VelocityLimit> & vel_limit)
{
  auto slow_down_traj = cruise_traj_points;
  if (slow_down_traj.empty()) {
    vel_limit = std::nullopt;
    return slow_down_traj;
  }

  for (const auto & obstacle : obstacles) {
    const auto start_idx = motion_utils::getDistanceIndex(
      slow_down_traj, obstacle.front_collision_distance);
    const auto end_idx = motion_utils::getDistanceIndex(
      slow_down_traj, obstacle.back_collision_distance);
    if (!start_idx || !end_idx) {
      continue;
    }
    const double target_velocity = std::max(0.0, obstacle.velocity - 0.5);
    for (size_t idx = *start_idx; idx <= *end_idx && idx < slow_down_traj.size(); ++idx) {
      slow_down_traj.at(idx).longitudinal_velocity_mps =
        std::min(slow_down_traj.at(idx).longitudinal_velocity_mps, target_velocity);
    }
  }

  vel_limit = std::nullopt;
  return slow_down_traj;
}

double PlannerInterface::calculateMarginFromObstacleOnCurve(
  const PlannerData & /*planner_data*/,
  const autoware::common_types::StopObstacle & stop_obstacle) const
{
  if (stop_obstacle.uuid == "goal_stop") {
    return longitudinal_info_.terminal_safe_distance_margin;
  }
  return longitudinal_info_.safe_distance_margin;
}
}  // namespace autoware::obstacle_cruise_planner
