#pragma once

#include "autoware/obstacle_cruise_planner/motion_utils.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_utils.hpp"

#include <common/common_types.hpp>

#include <optional>
#include <vector>

namespace autoware::obstacle_cruise_planner
{
struct LongitudinalInfo
{
  double safe_distance_margin{1.0};
  double terminal_safe_distance_margin{1.0};
  double hold_stop_velocity_threshold{0.1};
  double hold_stop_distance_threshold{0.5};
};

struct PlannerData
{
  std::vector<autoware::common_types::TrajectoryPoint> traj_points;
  autoware::common_types::Pose ego_pose;
  double ego_vel{0.0};
  double ego_acc{0.0};
  bool is_driving_forward{true};
};

struct BehaviorDeterminationParam
{
  double max_lat_margin_for_stop{0.8};
  double max_lat_margin_for_slow_down{1.2};
  double stop_distance_threshold{8.0};
  double slow_down_distance_margin{5.0};
  double obstacle_velocity_threshold_from_stop_to_cruise{3.5};
  double stop_obstacle_hold_time_threshold{1.0};
};

class PlannerInterface
{
public:
  explicit PlannerInterface(const LongitudinalInfo & longitudinal_info);
  std::vector<autoware::common_types::TrajectoryPoint> generateStopTrajectory(
    const PlannerData & planner_data,
    const std::vector<autoware::common_types::StopObstacle> & stop_obstacles);
  virtual std::vector<autoware::common_types::TrajectoryPoint> generateCruiseTrajectory(
    const PlannerData & planner_data,
    const std::vector<autoware::common_types::TrajectoryPoint> & cruise_traj_points,
    const std::vector<autoware::common_types::CruiseObstacle> & obstacles,
    std::optional<autoware::common_types::VelocityLimit> & vel_limit) = 0;
  std::vector<autoware::common_types::TrajectoryPoint> generateSlowDownTrajectory(
    const PlannerData & planner_data,
    const std::vector<autoware::common_types::TrajectoryPoint> & cruise_traj_points,
    const std::vector<autoware::common_types::SlowDownObstacle> & obstacles,
    std::optional<autoware::common_types::VelocityLimit> & vel_limit);

private:
  double calculateMarginFromObstacleOnCurve(
    const PlannerData & planner_data,
    const autoware::common_types::StopObstacle & stop_obstacle) const;

  LongitudinalInfo longitudinal_info_;
};
}  // namespace autoware::obstacle_cruise_planner
