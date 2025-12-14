#pragma once

#include "autoware/obstacle_cruise_planner/planner_interface.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"

#include <common/common_types.hpp>

#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace autoware::obstacle_cruise_planner
{
namespace ct = autoware::common_types;
using Polygon2d = std::vector<ct::Point>;

class ObstacleCruisePlannerNode
{
public:
  ObstacleCruisePlannerNode(
    const LongitudinalInfo & longitudinal_info,
    const BehaviorDeterminationParam & behavior_param);

  std::vector<ct::TrajectoryPoint> planTrajectory(
    const ct::Trajectory & trajectory, const double ego_vel, const double ego_acc,
    const std::vector<ct::PredictedObject> & predicted_objects,
    const std::optional<ct::TimeStamp> & predicted_objects_stamp = std::nullopt,
    const bool include_goal_stop = true);

private:
  PlannerData createPlannerData(
    const ct::Trajectory & trajectory, const double ego_vel, const double ego_acc) const;

  std::vector<ct::StopObstacle> createStopObstacles(
    const ct::Trajectory & trajectory, const bool include_goal_stop = true) const;

  std::vector<ct::Obstacle> convertToObstacles(
    const std::vector<ct::PredictedObject> & predicted_objects,
    const std::vector<ct::TrajectoryPoint> & traj_points) const;

  std::tuple<std::vector<ct::StopObstacle>, std::vector<ct::CruiseObstacle>,
             std::vector<ct::SlowDownObstacle>>
  determineEgoBehaviorAgainstPredictedObjectObstacles(
    const std::vector<ct::Obstacle> & obstacles,
    const std::vector<ct::TrajectoryPoint> & traj_points,
    const ct::TimeStamp & predicted_objects_stamp) const;

  std::optional<ct::StopObstacle> createStopObstacleForPredictedObject(
    const std::vector<ct::TrajectoryPoint> & traj_points, const ct::Obstacle & obstacle,
    const ct::TimeStamp & stamp) const;

  std::optional<ct::SlowDownObstacle> createSlowDownObstacleForPredictedObject(
    const std::vector<ct::TrajectoryPoint> & traj_points, const ct::Obstacle & obstacle) const;

  std::vector<Polygon2d> createOneStepPolygons(
    const std::vector<ct::TrajectoryPoint> & traj_points) const;

  void checkConsistency(
    const ct::TimeStamp & current_time,
    const std::optional<ct::TimeStamp> & predicted_objects_stamp,
    const std::vector<ct::PredictedObject> & predicted_objects,
    std::vector<ct::StopObstacle> & stop_obstacles);

  std::unique_ptr<PlannerInterface> planner_;
  LongitudinalInfo longitudinal_info_;
  BehaviorDeterminationParam behavior_param_;
  std::vector<ct::StopObstacle> prev_closest_stop_object_obstacles_;
};
}  // namespace autoware::obstacle_cruise_planner
