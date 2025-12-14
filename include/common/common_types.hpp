#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::common_types
{
// --------------------------------------------------------------------------
// 1) Time / Header
// --------------------------------------------------------------------------
struct TimeStamp
{
  std::int64_t sec{0};
  std::int64_t nsec{0};
};

struct Duration
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

struct Header
{
  std::uint32_t seq{0};
  TimeStamp stamp{};
  std::string frame_id;
};

// --------------------------------------------------------------------------
// 2) 기본 공간/자세 타입
// --------------------------------------------------------------------------
struct UUID
{
  std::uint8_t bytes[16]{};
};

struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Orientation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

// Autoware에서 Quaternion 과 Orientation 을 사실상 동일하게 쓰므로 alias 처리
using Quaternion = Orientation;

struct Pose
{
  Position position;
  Orientation orientation;
};

struct PointXYZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// --------------------------------------------------------------------------
// 3) Lanelet / Route 타입
// (structs removed since lanelet2 not required for ROS-less port)
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// 4) 벡터/속도 관련
// --------------------------------------------------------------------------
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct Polygon
{
  std::vector<Point> point;   // Autoware 옛 버전 메시지 스타일
};

struct Shape
{
  enum Type { BOUNDING_BOX, CYLINDER, POLYGON } type{BOUNDING_BOX};
  Polygon footprint;
  Vector3 dimensions;
};

// --------------------------------------------------------------------------
// 5) Path 계열 타입
// --------------------------------------------------------------------------
struct PathPoint
{
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double heading_rate_rps{0.0};
  bool is_final{false};
};

struct PathPointWithLaneId
{
  PathPoint point;
  std::vector<std::int64_t> lane_ids;
};

struct PathWithLaneId
{
  using SharedPtr = std::shared_ptr<PathWithLaneId>;

  Header header;
  std::vector<PathPointWithLaneId> points;
  std::vector<PointXYZ> left_bound;
  std::vector<PointXYZ> right_bound;
};

struct Path
{
  Header header;
  std::vector<PathPoint> points;
  std::vector<PointXYZ> left_bound;
  std::vector<PointXYZ> right_bound;
};

struct TrajectoryPoint
{
  Duration time_from_start{};
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double heading_rate_rps{0.0};
  double front_wheel_angle_rad{0.0};
  double rear_wheel_angle_rad{0.0};
};

struct Trajectory
{
  Header header;
  std::vector<TrajectoryPoint> points;
};

// --------------------------------------------------------------------------
// 6) Covariance 포함 타입
// --------------------------------------------------------------------------
struct PoseWithCovariance
{
  Pose pose;
  std::array<double, 36> covariance{}; // 6x6 matrix
};

struct TwistWithCovariance
{
  Vector3 linear;                 // m/s
  Vector3 angular;                // rad/s
  std::array<double, 36> covariance{}; // 6x6 matrix
};

// --------------------------------------------------------------------------
// 7) Odometry / Accel 계열
// --------------------------------------------------------------------------
struct Odometry
{
  using ConstSharedPtr = std::shared_ptr<const Odometry>;

  Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

struct KinematicState
{
  Header header;
  Pose pose;

  struct TwistData
  {
    double linear_x{0.0};
    double linear_y{0.0};
    double linear_z{0.0};
    double angular_x{0.0};
    double angular_y{0.0};
    double angular_z{0.0};
  } twist;
};

struct Accel
{
  Vector3 linear;   // m/s²
  Vector3 angular;  // rad/s²
};

struct AccelWithCovariance
{
  Accel accel;
  std::array<double, 36> covariance{}; // 6x6 matrix
};

struct AccelWithCovarianceStamped
{
  using ConstSharedPtr = std::shared_ptr<const AccelWithCovarianceStamped>;

  Header header;
  AccelWithCovariance accel;
};

// --------------------------------------------------------------------------
// 8) Objects / Prediction
// (Simplified versions of predicted data are defined near the bottom.)
// --------------------------------------------------------------------------
struct OperationModeState
{
  using ConstSharedPtr = std::shared_ptr<const OperationModeState>;

  enum class Mode : std::uint8_t
  {
    UNKNOWN = 0,
    MANUAL,
    AUTONOMOUS,
  };

  TimeStamp stamp{};
  Mode mode{Mode::UNKNOWN};
};

struct VelocityLimit
{
  using ConstSharedPtr = std::shared_ptr<const VelocityLimit>;

  TimeStamp stamp{};
  double max_velocity{0.0};
};

// --------------------------------------------------------------------------
// 10) PoseStamped / PoseWithUuid
// --------------------------------------------------------------------------
struct PoseWithUuidStamped
{
  TimeStamp stamp{};
  Pose pose{};
  UUID uuid{};
};

struct PoseStamped
{
  Header header;
  Pose pose;
};

// --------------------------------------------------------------------------
// 11) Drivable Area / BehaviorModuleOutput (단순화 버전)
// --------------------------------------------------------------------------
struct DrivableAreaInfo
{
  std::vector<PathPoint> drivable_points;
};

struct BehaviorModuleOutput
{
  BehaviorModuleOutput() = default;
  PathWithLaneId path{};
  PathWithLaneId reference_path{};
  std::optional<PoseWithUuidStamped> modified_goal{};
  DrivableAreaInfo drivable_area_info;
};

// --------------------------------------------------------------------------
// 12) Utility 타입
// --------------------------------------------------------------------------
struct PoseWithDetail
{
  Pose pose;
  std::string detail;
  explicit PoseWithDetail(const Pose & p, const std::string & d = "") : pose(p), detail(d) {}
};

using PoseWithDetailOpt = std::optional<PoseWithDetail>;

struct PointWithStamp
{
  Point point;
  double stamp{0.0};
};

struct PredictedPath
{
  std::vector<Point> path;
  double time_step{0.1};
};

struct PredictedObject
{
  std::string uuid;
  double distance_along_traj{0.0};
  double lateral_distance{0.0};
  double velocity{0.0};
  Pose pose;
  std::vector<PredictedPath> predicted_paths;
};

struct Obstacle
{
  std::string uuid;
  double dist_to_collision{0.0};
  double lateral_distance{0.0};
  double velocity{0.0};
  Pose pose;
};

struct StopObstacle
{
  std::string uuid;
  double dist_to_collide_on_decimated_traj{0.0};
  Point collision_point{};
  double velocity{0.0};
  TimeStamp stamp{};
};

struct SlowDownObstacle
{
  std::string uuid;
  double precise_lat_dist{0.0};
  double front_collision_distance{0.0};
  double back_collision_distance{0.0};
  Point front_collision_point{};
  Point back_collision_point{};
  double velocity{0.0};
};

struct CruiseObstacle
{
  std::string uuid;
  std::vector<PointWithStamp> collision_points;
};

}  // namespace autoware::common_types
