#pragma once

#include "autoware/obstacle_cruise_planner/planner_interface.hpp"

#include <optional>
#include <vector>

namespace autoware::obstacle_cruise_planner
{
class OptimizationBasedPlanner : public PlannerInterface
{
public:
  explicit OptimizationBasedPlanner(const LongitudinalInfo & longitudinal_info);
  std::vector<autoware::common_types::TrajectoryPoint> generateCruiseTrajectory(
    const PlannerData & planner_data, const std::vector<autoware::common_types::TrajectoryPoint> & stop_traj_points,
    const std::vector<autoware::common_types::CruiseObstacle> & obstacles,
    std::optional<autoware::common_types::VelocityLimit> & vel_limit) override;
};
}  // namespace autoware::obstacle_cruise_planner
