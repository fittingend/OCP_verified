#include "autoware/obstacle_cruise_planner/obstacle_cruise_planner_node.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_utils.hpp"
#include "autoware/obstacle_cruise_planner/optimization_based_planner.hpp"
#include "autoware/obstacle_cruise_planner/motion_utils.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

namespace autoware::obstacle_cruise_planner
{
namespace ct = autoware::common_types;

namespace
{
double calcTimeDiffSeconds(const ct::TimeStamp & from, const ct::TimeStamp & to)
{
  const double sec_diff = static_cast<double>(to.sec - from.sec);
  const double nsec_diff = static_cast<double>(to.nsec - from.nsec) * 1e-9;
  return sec_diff + nsec_diff;
}
}  // namespace
ObstacleCruisePlannerNode::ObstacleCruisePlannerNode(
  const LongitudinalInfo & longitudinal_info,
  const BehaviorDeterminationParam & behavior_param)
: planner_(std::make_unique<OptimizationBasedPlanner>(longitudinal_info)),
  longitudinal_info_(longitudinal_info),
  behavior_param_(behavior_param)
{
}

std::vector<ct::TrajectoryPoint> ObstacleCruisePlannerNode::planTrajectory(
  const ct::Trajectory & trajectory, const double ego_vel, const double ego_acc,
  const std::vector<ct::PredictedObject> & predicted_objects,
  const std::optional<ct::TimeStamp> & predicted_objects_stamp, const bool include_goal_stop)
{
  const ct::TimeStamp current_stamp =
    predicted_objects_stamp ? *predicted_objects_stamp : trajectory.header.stamp;
  const auto obstacles = convertToObstacles(predicted_objects, trajectory.points);
  auto [stop_obstacles, cruise_obstacles, slow_down_obstacles] =
    determineEgoBehaviorAgainstPredictedObjectObstacles(
      obstacles, trajectory.points, current_stamp);
  checkConsistency(current_stamp, predicted_objects_stamp, predicted_objects, stop_obstacles);
  const auto planner_data = createPlannerData(trajectory, ego_vel, ego_acc);

  const auto goal_obstacles = createStopObstacles(trajectory, include_goal_stop);
  stop_obstacles.insert(stop_obstacles.end(), goal_obstacles.begin(), goal_obstacles.end());

  const auto stop_traj = planner_->generateStopTrajectory(planner_data, stop_obstacles);

  std::optional<ct::VelocityLimit> cruise_vel_limit;
  const auto cruise_traj = planner_->generateCruiseTrajectory(
    planner_data, stop_traj, cruise_obstacles, cruise_vel_limit);

  std::optional<ct::VelocityLimit> slow_vel_limit;
  const auto slow_down_traj = planner_->generateSlowDownTrajectory(
    planner_data, cruise_traj, slow_down_obstacles, slow_vel_limit);

  return slow_down_traj;
}

PlannerData ObstacleCruisePlannerNode::createPlannerData(
  const ct::Trajectory & trajectory, const double ego_vel, const double ego_acc) const
{
  PlannerData planner_data;
  planner_data.traj_points = trajectory.points;
  planner_data.ego_pose = trajectory.points.empty() ? ct::Pose{} : trajectory.points.front().pose;
  planner_data.ego_vel = ego_vel;
  planner_data.ego_acc = ego_acc;
  planner_data.is_driving_forward = true;
  return planner_data;
}

std::vector<ct::StopObstacle> ObstacleCruisePlannerNode::createStopObstacles(
  const ct::Trajectory & trajectory, const bool include_goal_stop) const
{
  if (!include_goal_stop || trajectory.points.empty()) {
    return {};
  }

  const size_t last_idx = trajectory.points.size() - 1;
  const double total_length =
    motion_utils::calcSignedArcLength(trajectory.points, 0, last_idx);

  ct::StopObstacle stop_obstacle;
  stop_obstacle.uuid = "goal_stop";
  stop_obstacle.dist_to_collide_on_decimated_traj = total_length;
  const auto & goal_position = trajectory.points.at(last_idx).pose.position;
  stop_obstacle.collision_point.x = goal_position.x;
  stop_obstacle.collision_point.y = goal_position.y;
  stop_obstacle.collision_point.z = goal_position.z;
  stop_obstacle.velocity = 0.0;
  stop_obstacle.stamp = trajectory.header.stamp;

  return {stop_obstacle};
}

std::vector<ct::Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const std::vector<ct::PredictedObject> & predicted_objects,
  const std::vector<ct::TrajectoryPoint> & traj_points) const
{
  std::vector<ct::Obstacle> obstacles;
  for (const auto & object : predicted_objects) {
    double dist_along_traj = object.distance_along_traj;
    if (dist_along_traj <= 0.0 && !traj_points.empty()) {
      dist_along_traj =
        motion_utils::calcSignedArcLengthToPose(traj_points, object.pose);
    }
    if (dist_along_traj < 0.0) {
      continue;
    }
    if (std::abs(object.lateral_distance) > behavior_param_.max_lat_margin_for_slow_down) {
      continue;
    }
    ct::Obstacle obstacle;
    obstacle.uuid = object.uuid;
    obstacle.dist_to_collision = dist_along_traj;
    obstacle.lateral_distance = object.lateral_distance;
    obstacle.velocity = object.velocity;
    obstacle.pose = object.pose;
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

std::tuple<std::vector<ct::StopObstacle>, std::vector<ct::CruiseObstacle>,
           std::vector<ct::SlowDownObstacle>>
ObstacleCruisePlannerNode::determineEgoBehaviorAgainstPredictedObjectObstacles(
  const std::vector<ct::Obstacle> & obstacles,
  const std::vector<ct::TrajectoryPoint> & traj_points,
  const ct::TimeStamp & predicted_objects_stamp) const
{
  std::vector<ct::StopObstacle> stop_obstacles;
  std::vector<ct::CruiseObstacle> cruise_obstacles;
  std::vector<ct::SlowDownObstacle> slow_down_obstacles;

  for (const auto & obstacle : obstacles) {
    if (const auto stop_obstacle = createStopObstacleForPredictedObject(
          traj_points, obstacle, predicted_objects_stamp)) {
      stop_obstacles.push_back(*stop_obstacle);
      continue;
    }
    if (const auto slow_down_obstacle = createSlowDownObstacleForPredictedObject(traj_points, obstacle)) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
    }
  }

  return {stop_obstacles, cruise_obstacles, slow_down_obstacles};
}

std::optional<ct::StopObstacle> ObstacleCruisePlannerNode::createStopObstacleForPredictedObject(
  const std::vector<ct::TrajectoryPoint> & traj_points, const ct::Obstacle & obstacle,
  const ct::TimeStamp & stamp) const
{
  if (obstacle.dist_to_collision > behavior_param_.stop_distance_threshold) {
    return std::nullopt;
  }
  if (std::abs(obstacle.lateral_distance) > behavior_param_.max_lat_margin_for_stop) {
    return std::nullopt;
  }

  ct::StopObstacle stop_obstacle;
  stop_obstacle.uuid = obstacle.uuid;
  stop_obstacle.velocity = obstacle.velocity;
  stop_obstacle.stamp = stamp;
  stop_obstacle.dist_to_collide_on_decimated_traj = obstacle.dist_to_collision;
  stop_obstacle.collision_point.x = obstacle.pose.position.x;
  stop_obstacle.collision_point.y = obstacle.pose.position.y;
  stop_obstacle.collision_point.z = obstacle.pose.position.z;

  if (const auto collision_info = polygon_utils::getCollisionPoint(traj_points, stop_obstacle)) {
    stop_obstacle.collision_point = collision_info->first;
    stop_obstacle.dist_to_collide_on_decimated_traj = collision_info->second;
  }

  return stop_obstacle;
}

std::optional<ct::SlowDownObstacle> ObstacleCruisePlannerNode::createSlowDownObstacleForPredictedObject(
  const std::vector<ct::TrajectoryPoint> & /*traj_points*/, const ct::Obstacle & obstacle) const
{
  const double slow_down_max_dist =
    behavior_param_.stop_distance_threshold + behavior_param_.slow_down_distance_margin;
  if (obstacle.dist_to_collision > slow_down_max_dist) {
    return std::nullopt;
  }
  if (std::abs(obstacle.lateral_distance) > behavior_param_.max_lat_margin_for_slow_down) {
    return std::nullopt;
  }

  ct::SlowDownObstacle slow_down_obstacle;
  slow_down_obstacle.uuid = obstacle.uuid;
  slow_down_obstacle.precise_lat_dist = obstacle.lateral_distance;
  slow_down_obstacle.front_collision_distance =
    std::max(0.0, obstacle.dist_to_collision - behavior_param_.slow_down_distance_margin);
  slow_down_obstacle.back_collision_distance =
    obstacle.dist_to_collision + behavior_param_.slow_down_distance_margin;
  slow_down_obstacle.front_collision_point.x = obstacle.pose.position.x;
  slow_down_obstacle.front_collision_point.y = obstacle.pose.position.y;
  slow_down_obstacle.front_collision_point.z = obstacle.pose.position.z;
  slow_down_obstacle.back_collision_point.x = obstacle.pose.position.x;
  slow_down_obstacle.back_collision_point.y = obstacle.pose.position.y;
  slow_down_obstacle.back_collision_point.z = obstacle.pose.position.z;
  slow_down_obstacle.velocity = obstacle.velocity;
  return slow_down_obstacle;
}

void ObstacleCruisePlannerNode::checkConsistency(
  const ct::TimeStamp & current_time,
  const std::optional<ct::TimeStamp> & predicted_objects_stamp,
  const std::vector<ct::PredictedObject> & predicted_objects,
  std::vector<ct::StopObstacle> & stop_obstacles)
{
  const ct::TimeStamp object_stamp =
    predicted_objects_stamp ? *predicted_objects_stamp : current_time;
  for (const auto & prev_stop_obstacle : prev_closest_stop_object_obstacles_) {
    const auto predicted_it = std::find_if(
      predicted_objects.begin(), predicted_objects.end(),
      [&prev_stop_obstacle, &object_stamp](const ct::PredictedObject & po) {
      return po.uuid == prev_stop_obstacle.uuid;
      });
    if (predicted_it == predicted_objects.end()) {
      continue;
    }
    const auto existed = std::any_of(
      stop_obstacles.begin(), stop_obstacles.end(),
      [&prev_stop_obstacle](const ct::StopObstacle & so) {
        return so.uuid == prev_stop_obstacle.uuid;
      });
    if (existed) {
      continue;
    }
    const double elapsed_time = std::max(
      0.0, calcTimeDiffSeconds(prev_stop_obstacle.stamp, current_time));
    if (
      predicted_it->velocity < behavior_param_.obstacle_velocity_threshold_from_stop_to_cruise &&
      elapsed_time < behavior_param_.stop_obstacle_hold_time_threshold) {
      stop_obstacles.push_back(prev_stop_obstacle);
    }
  }

  prev_closest_stop_object_obstacles_ =
    obstacle_utils::getClosestStopObstacles(stop_obstacles);
}

std::vector<Polygon2d> ObstacleCruisePlannerNode::createOneStepPolygons(
  const std::vector<ct::TrajectoryPoint> & /*traj_points*/) const
{
  return {};
}
}  // namespace autoware::obstacle_cruise_planner
