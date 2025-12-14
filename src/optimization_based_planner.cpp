#include "autoware/obstacle_cruise_planner/optimization_based_planner.hpp"

#include <algorithm>
#include <optional>

namespace autoware::obstacle_cruise_planner
{
OptimizationBasedPlanner::OptimizationBasedPlanner(const LongitudinalInfo & longitudinal_info)
: PlannerInterface(longitudinal_info)
{
}

std::vector<autoware::common_types::TrajectoryPoint> OptimizationBasedPlanner::generateCruiseTrajectory(
  const PlannerData & /*planner_data*/,
  const std::vector<autoware::common_types::TrajectoryPoint> & stop_traj_points,
  const std::vector<autoware::common_types::CruiseObstacle> & /*obstacles*/,
  std::optional<autoware::common_types::VelocityLimit> & vel_limit)
{
  vel_limit = std::nullopt;
  return stop_traj_points;
}
}  // namespace autoware::obstacle_cruise_planner
