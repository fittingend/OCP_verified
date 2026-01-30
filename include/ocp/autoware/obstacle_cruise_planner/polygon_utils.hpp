#pragma once

#include <common/common_types.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <optional>
#include <vector>

namespace autoware::obstacle_cruise_planner::polygon_utils
{
namespace ct = autoware::common_types;
using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d = boost::geometry::model::polygon<Point2d>;

Polygon2d toPolygon2d(const ct::Pose & pose, const ct::Shape & shape);

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & traj_polygons, const ct::Obstacle & obstacle,
  const bool is_driving_forward, const ct::VehicleInfo & vehicle_info);

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points, const size_t collision_idx,
  const std::vector<ct::PointWithStamp> & collision_points, const bool is_driving_forward,
  const ct::VehicleInfo & vehicle_info);

std::optional<std::pair<ct::Point, double>> getCollisionPointFromPredictedPath(
  const std::vector<ct::TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & traj_polygons, const ct::PredictedPath & predicted_path,
  const ct::Shape & shape, const bool is_driving_forward, const ct::VehicleInfo & vehicle_info,
  const double max_lat_dist);

}  // namespace autoware::obstacle_cruise_planner::polygon_utils
