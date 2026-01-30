#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"

#include "autoware/obstacle_cruise_planner/motion_utils.hpp"

#include <boost/geometry.hpp>
#include <utility>

namespace autoware::obstacle_cruise_planner::polygon_utils
{
namespace ct = autoware::common_types;

namespace
{
double yawFromOrientation(const ct::Orientation & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

ct::Pose calcOffsetPose(const ct::Pose & pose, const double x, const double y)
{
  ct::Pose out = pose;
  const double yaw = yawFromOrientation(pose.orientation);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  out.position.x += x * cos_y - y * sin_y;
  out.position.y += x * sin_y + y * cos_y;
  return out;
}

ct::Point inverseTransformPoint(const ct::Point & point, const ct::Pose & pose)
{
  const double yaw = yawFromOrientation(pose.orientation);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  const double dx = point.x - pose.position.x;
  const double dy = point.y - pose.position.y;
  ct::Point out;
  out.x = dx * cos_y + dy * sin_y;
  out.y = -dx * sin_y + dy * cos_y;
  out.z = point.z - pose.position.z;
  return out;
}

Polygon2d toPolygon2dImpl(const ct::Pose & pose, const ct::Shape & shape)
{
  Polygon2d poly;
  double length = shape.dimensions.x;
  double width = shape.dimensions.y;
  if (shape.type == ct::Shape::POLYGON && !shape.footprint.point.empty()) {
    const double yaw = yawFromOrientation(pose.orientation);
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);
    for (const auto & p : shape.footprint.point) {
      const double x = pose.position.x + p.x * cos_y - p.y * sin_y;
      const double y = pose.position.y + p.x * sin_y + p.y * cos_y;
      boost::geometry::append(poly.outer(), Point2d(x, y));
    }
  } else {
    const double front = length * 0.5;
    const double rear = -length * 0.5;
    const double half_width = width * 0.5;
    const double yaw = yawFromOrientation(pose.orientation);
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);
    std::array<ct::Point, 4> corners{
      ct::Point{front, half_width, 0.0},
      ct::Point{front, -half_width, 0.0},
      ct::Point{rear, -half_width, 0.0},
      ct::Point{rear, half_width, 0.0},
    };
    for (const auto & corner : corners) {
      const double x = pose.position.x + corner.x * cos_y - corner.y * sin_y;
      const double y = pose.position.y + corner.x * sin_y + corner.y * cos_y;
      boost::geometry::append(poly.outer(), Point2d(x, y));
    }
  }
  if (!poly.outer().empty()) {
    boost::geometry::append(poly.outer(), poly.outer().front());
  }
  boost::geometry::correct(poly);
  return poly;
}

std::optional<std::pair<size_t, std::vector<ct::PointWithStamp>>> getCollisionIndex(
  const std::vector<ct::TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const ct::Pose & object_pose, const ct::Shape & object_shape,
  const double max_dist = std::numeric_limits<double>::max())
{
  const auto obj_polygon = toPolygon2dImpl(object_pose, object_shape);
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist = motion_utils::calcDistance(traj_points.at(i).pose, object_pose);
    if (approximated_dist > max_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    std::vector<ct::PointWithStamp> collision_geom_points;
    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;
        for (const auto & collision_point : collision_polygon.outer()) {
          ct::PointWithStamp collision_geom_point;
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
          collision_geom_point.point.z = traj_points.at(i).pose.position.z;
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      return std::make_pair(i, collision_geom_points);
    }
  }
  return std::nullopt;
}
}  // namespace

Polygon2d toPolygon2d(const ct::Pose & pose, const ct::Shape & shape)
{
  return toPolygon2dImpl(pose, shape);
}

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const ct::Obstacle & obstacle, const bool is_driving_forward, const ct::VehicleInfo & vehicle_info)
{
  const auto collision_info =
    getCollisionIndex(traj_points, traj_polygons, obstacle.pose, obstacle.shape);
  if (!collision_info) {
    return std::nullopt;
  }

  const double x_diff_to_bumper =
    is_driving_forward ? vehicle_info.max_longitudinal_offset_m : vehicle_info.min_longitudinal_offset_m;
  const auto bumper_pose = calcOffsetPose(traj_points.at(collision_info->first).pose, x_diff_to_bumper, 0.0);

  std::optional<double> max_collision_length{};
  std::optional<ct::Point> max_collision_point{};
  for (const auto & poly_vertex : collision_info->second) {
    const double dist_from_bumper = std::abs(inverseTransformPoint(poly_vertex.point, bumper_pose).x);
    if (!max_collision_length || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  if (!max_collision_point || !max_collision_length) {
    return std::nullopt;
  }
  const double base_s = motion_utils::calcSignedArcLength(traj_points, 0, collision_info->first);
  return std::make_pair(*max_collision_point, base_s - *max_collision_length);
}

std::optional<std::pair<ct::Point, double>> getCollisionPoint(
  const std::vector<ct::TrajectoryPoint> & traj_points, const size_t collision_idx,
  const std::vector<ct::PointWithStamp> & collision_points, const bool is_driving_forward,
  const ct::VehicleInfo & vehicle_info)
{
  const double x_diff_to_bumper =
    is_driving_forward ? vehicle_info.max_longitudinal_offset_m : vehicle_info.min_longitudinal_offset_m;
  const auto bumper_pose = calcOffsetPose(traj_points.at(collision_idx).pose, x_diff_to_bumper, 0.0);

  std::optional<double> max_collision_length{};
  std::optional<ct::Point> max_collision_point{};
  for (const auto & poly_vertex : collision_points) {
    const double dist_from_bumper = std::abs(inverseTransformPoint(poly_vertex.point, bumper_pose).x);
    if (!max_collision_length || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  if (!max_collision_point || !max_collision_length) {
    return std::nullopt;
  }
  const double base_s = motion_utils::calcSignedArcLength(traj_points, 0, collision_idx);
  return std::make_pair(*max_collision_point, base_s - *max_collision_length);
}

std::optional<std::pair<ct::Point, double>> getCollisionPointFromPredictedPath(
  const std::vector<ct::TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const ct::PredictedPath & predicted_path, const ct::Shape & shape, const bool is_driving_forward,
  const ct::VehicleInfo & vehicle_info, const double max_lat_dist)
{
  for (const auto & point : predicted_path.path) {
    ct::Pose pose{};
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    const auto collision_info = getCollisionIndex(
      traj_points, traj_polygons, pose, shape, max_lat_dist);
    if (collision_info) {
      return getCollisionPoint(
        traj_points, collision_info->first, collision_info->second, is_driving_forward, vehicle_info);
    }
  }
  return std::nullopt;
}

}  // namespace autoware::obstacle_cruise_planner::polygon_utils
