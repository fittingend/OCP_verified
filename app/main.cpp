#include "autoware/obstacle_cruise_planner/motion_utils.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_cruise_planner_node.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cctype>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <utility>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace ct = autoware::common_types;
namespace acp = autoware::obstacle_cruise_planner;
namespace fs = std::filesystem;

namespace
{
std::string trim(const std::string & str)
{
  const auto first = str.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = str.find_last_not_of(" \t\r\n");
  return str.substr(first, last - first + 1);
}

std::map<std::string, std::string> loadConfig(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open config file: " + path);
  }
  std::map<std::string, std::string> config;
  std::string line;
  while (std::getline(ifs, line)) {
    const auto comment_pos = line.find_first_of('#');
    const auto effective_line = trim(line.substr(0, comment_pos));
    if (effective_line.empty()) {
      continue;
    }
    const auto sep = effective_line.find(':');
    if (sep == std::string::npos) {
      continue;
    }
    const auto key = trim(effective_line.substr(0, sep));
    const auto value = trim(effective_line.substr(sep + 1));
    if (!key.empty()) {
      config[key] = value;
    }
  }
  return config;
}

ct::Trajectory createStraightTrajectory(const double length, const size_t points)
{
  ct::Trajectory trajectory;
  trajectory.points.reserve(points);
  for (size_t i = 0; i < points; ++i) {
    const double ratio = static_cast<double>(i) / (points - 1);
    const double distance = length * ratio;
    ct::TrajectoryPoint point;
    point.pose.position.x = distance;
    point.pose.position.y = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0;
    point.time_from_start.sec = static_cast<std::int32_t>(i);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

ct::Trajectory createLeftTurnTrajectory(const double radius, const size_t points)
{
  ct::Trajectory trajectory;
  trajectory.points.reserve(points);
  const double total_angle = M_PI / 4.0;
  for (size_t i = 0; i < points; ++i) {
    const double ratio = static_cast<double>(i) / (points - 1);
    const double angle = total_angle * ratio;
    ct::TrajectoryPoint point;
    point.pose.position.x = radius * std::sin(angle);
    point.pose.position.y = radius * (1.0 - std::cos(angle));
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0;
    point.time_from_start.sec = static_cast<std::int32_t>(i);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

ct::PredictedObject createStaticObstacle(const double distance)
{
  ct::PredictedObject object;
  object.uuid = "obstacle";
  object.distance_along_traj = distance;
  object.lateral_distance = 0.0;
  object.velocity = 0.0;
  object.pose.position.x = distance;
  object.pose.position.y = 0.0;
  ct::PredictedPath predicted_path;
  predicted_path.path.push_back(ct::Point{distance, 0.0, 0.0});
  object.predicted_paths.push_back(predicted_path);
  return object;
}

struct JsonValue
{
  enum class Type {
    Null,
    Bool,
    Number,
    String,
    Array,
    Object,
  };

  using array_t = std::vector<JsonValue>;
  using object_t = std::map<std::string, JsonValue>;

  JsonValue() = default;
  JsonValue(bool value) : type(Type::Bool), value(value) {}
  JsonValue(double value) : type(Type::Number), value(value) {}
  JsonValue(std::string value) : type(Type::String), value(std::move(value)) {}
  JsonValue(const array_t & value) : type(Type::Array), value(value) {}
  JsonValue(array_t && value) : type(Type::Array), value(std::move(value)) {}
  JsonValue(const object_t & value) : type(Type::Object), value(value) {}
  JsonValue(object_t && value) : type(Type::Object), value(std::move(value)) {}

  bool isNull() const { return type == Type::Null; }
  bool isBool() const { return type == Type::Bool; }
  bool isNumber() const { return type == Type::Number; }
  bool isString() const { return type == Type::String; }
  bool isArray() const { return type == Type::Array; }
  bool isObject() const { return type == Type::Object; }

  const array_t & asArray() const
  {
    if (!isArray()) {
      throw std::runtime_error("JSON value is not an array");
    }
    return std::get<array_t>(value);
  }

  const object_t & asObject() const
  {
    if (!isObject()) {
      throw std::runtime_error("JSON value is not an object");
    }
    return std::get<object_t>(value);
  }

  double asNumber() const
  {
    if (!isNumber()) {
      throw std::runtime_error("JSON value is not a number");
    }
    return std::get<double>(value);
  }

  const std::string & asString() const
  {
    if (!isString()) {
      throw std::runtime_error("JSON value is not a string");
    }
    return std::get<std::string>(value);
  }

  std::size_t arraySize() const
  {
    return isArray() ? std::get<array_t>(value).size() : 0u;
  }

  const JsonValue & operator[](const std::string & key) const
  {
    if (!isObject()) {
      return nullValue();
    }
    const auto & obj = std::get<object_t>(value);
    const auto it = obj.find(key);
    if (it == obj.end()) {
      return nullValue();
    }
    return it->second;
  }

  const JsonValue & operator[](const std::size_t idx) const
  {
    if (!isArray()) {
      return nullValue();
    }
    const auto & arr = std::get<array_t>(value);
    if (idx >= arr.size()) {
      return nullValue();
    }
    return arr[idx];
  }

  static const JsonValue & nullValue()
  {
    static JsonValue value;
    return value;
  }

  Type type{Type::Null};
  std::variant<std::monostate, bool, double, std::string, array_t, object_t> value;
};

class JsonParser
{
public:
  explicit JsonParser(const std::string & text) : ptr_(text.c_str()), end_(text.c_str() + text.size()) {}

  JsonValue parse()
  {
    skipWhitespace();
    const auto value = parseValue();
    skipWhitespace();
    return value;
  }

private:
  void skipWhitespace()
  {
    while (ptr_ < end_ && std::isspace(static_cast<unsigned char>(*ptr_))) {
      ++ptr_;
    }
  }

  char peek() const
  {
    return ptr_ < end_ ? *ptr_ : '\0';
  }

  char consume()
  {
    return ptr_ < end_ ? *ptr_++ : '\0';
  }

  JsonValue parseValue()
  {
    skipWhitespace();
    const char ch = peek();
    if (ch == '{') {
      return parseObject();
    }
    if (ch == '[') {
      return parseArray();
    }
    if (ch == '"') {
      return JsonValue(parseString());
    }
    if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+') {
      return JsonValue(parseNumber());
    }
    if (std::strncmp(ptr_, "true", 4) == 0) {
      ptr_ += 4;
      return JsonValue(true);
    }
    if (std::strncmp(ptr_, "false", 5) == 0) {
      ptr_ += 5;
      return JsonValue(false);
    }
    if (std::strncmp(ptr_, "null", 4) == 0) {
      ptr_ += 4;
      return JsonValue();
    }
    throw std::runtime_error("invalid JSON value");
  }

  JsonValue parseObject()
  {
    expect('{');
    JsonValue::object_t object;
    skipWhitespace();
    if (peek() == '}') {
      consume();
      return JsonValue(std::move(object));
    }
    while (true) {
      skipWhitespace();
      const auto key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();
      const auto value = parseValue();
      object.emplace(key, std::move(value));
      skipWhitespace();
      const char ch = consume();
      if (ch == '}') {
        break;
      }
      if (ch != ',') {
        throw std::runtime_error("invalid JSON object");
      }
    }
    return JsonValue(std::move(object));
  }

  JsonValue parseArray()
  {
    expect('[');
    JsonValue::array_t array;
    skipWhitespace();
    if (peek() == ']') {
      consume();
      return JsonValue(std::move(array));
    }
    while (true) {
      skipWhitespace();
      array.emplace_back(parseValue());
      skipWhitespace();
      const char ch = consume();
      if (ch == ']') {
        break;
      }
      if (ch != ',') {
        throw std::runtime_error("invalid JSON array");
      }
    }
    return JsonValue(std::move(array));
  }

  std::string parseString()
  {
    expect('"');
    std::string result;
    while (true) {
      if (ptr_ >= end_) {
        throw std::runtime_error("unterminated JSON string");
      }
      const char ch = consume();
      if (ch == '"') {
        break;
      }
      if (ch == '\\') {
        if (ptr_ >= end_) {
          throw std::runtime_error("invalid escape sequence");
        }
        const char esc = consume();
        switch (esc) {
          case '"': result.push_back('"'); break;
          case '\\': result.push_back('\\'); break;
          case '/': result.push_back('/'); break;
          case 'b': result.push_back('\b'); break;
          case 'f': result.push_back('\f'); break;
          case 'n': result.push_back('\n'); break;
          case 'r': result.push_back('\r'); break;
          case 't': result.push_back('\t'); break;
          default: result.push_back(esc); break;
        }
        continue;
      }
      result.push_back(ch);
    }
    return result;
  }

  double parseNumber()
  {
    const char * start = ptr_;
    if (*ptr_ == '+' || *ptr_ == '-') {
      ++ptr_;
    }
    while (ptr_ < end_ && std::isdigit(static_cast<unsigned char>(*ptr_))) {
      ++ptr_;
    }
    if (ptr_ < end_ && *ptr_ == '.') {
      ++ptr_;
      while (ptr_ < end_ && std::isdigit(static_cast<unsigned char>(*ptr_))) {
        ++ptr_;
      }
    }
    if (ptr_ < end_ && (*ptr_ == 'e' || *ptr_ == 'E')) {
      ++ptr_;
      if (ptr_ < end_ && (*ptr_ == '+' || *ptr_ == '-')) {
        ++ptr_;
      }
      while (ptr_ < end_ && std::isdigit(static_cast<unsigned char>(*ptr_))) {
        ++ptr_;
      }
    }
    const std::string token(start, ptr_);
    try {
      return std::stod(token);
    } catch (const std::exception &) {
      throw std::runtime_error("invalid JSON number");
    }
  }

  void expect(char expected)
  {
    const char ch = consume();
    if (ch != expected) {
      throw std::runtime_error("unexpected JSON character");
    }
  }

  const char * ptr_{nullptr};
  const char * end_{nullptr};
};

JsonValue parseJsonLine(const std::string & line)
{
  JsonParser parser(line);
  return parser.parse();
}

std::optional<JsonValue> loadLastMessage(const fs::path & path)
{
  std::ifstream ifs(path);
  if (!ifs) {
    return std::nullopt;
  }
  std::string line;
  JsonValue last;
  bool has_value = false;
  while (std::getline(ifs, line)) {
    const auto doc = parseJsonLine(line);
    const auto & message = doc["message"];
    if (!message.isNull()) {
      last = message;
      has_value = true;
    }
  }
  if (!has_value) {
    return std::nullopt;
  }
  return last;
}

ct::Point parsePoint(const JsonValue & json)
{
  ct::Point point;
  point.x = json["x"].isNumber() ? json["x"].asNumber() : 0.0;
  point.y = json["y"].isNumber() ? json["y"].asNumber() : 0.0;
  point.z = json["z"].isNumber() ? json["z"].asNumber() : 0.0;
  return point;
}

ct::Orientation parseOrientation(const JsonValue & json)
{
  ct::Orientation orientation;
  orientation.x = json["x"].isNumber() ? json["x"].asNumber() : 0.0;
  orientation.y = json["y"].isNumber() ? json["y"].asNumber() : 0.0;
  orientation.z = json["z"].isNumber() ? json["z"].asNumber() : 0.0;
  orientation.w = json["w"].isNumber() ? json["w"].asNumber() : 1.0;
  return orientation;
}

ct::Pose parsePose(const JsonValue & payload)
{
  ct::Pose pose;
  const JsonValue * pose_node = nullptr;
  if (payload["kinematics"].isObject()) {
    const auto & kinematics = payload["kinematics"];
    if (kinematics["initial_pose_with_covariance"].isObject() &&
        kinematics["initial_pose_with_covariance"]["pose"].isObject()) {
      pose_node = &kinematics["initial_pose_with_covariance"]["pose"];
    }
  }
  if (payload["pose"].isObject() && payload["pose"]["pose"].isObject()) {
    pose_node = &payload["pose"]["pose"];
  } else if (payload["pose"].isObject()) {
    pose_node = &payload["pose"];
  } else if (payload["position"].isObject()) {
    pose_node = &payload;
  }
  if (pose_node && pose_node->isObject()) {
    const auto & position = (*pose_node)["position"];
    const auto & orientation = (*pose_node)["orientation"];
    if (position.isObject()) {
      const auto pt = parsePoint(position);
      pose.position.x = pt.x;
      pose.position.y = pt.y;
      pose.position.z = pt.z;
    }
    if (orientation.isObject()) {
      pose.orientation = parseOrientation(orientation);
    }
  }
  return pose;
}

ct::Duration parseDuration(const JsonValue & json)
{
  ct::Duration duration;
  if (json["sec"].isNumber()) {
    duration.sec = static_cast<std::int32_t>(json["sec"].asNumber());
  }
  if (json["nanosec"].isNumber()) {
    duration.nanosec = static_cast<std::uint32_t>(json["nanosec"].asNumber());
  }
  return duration;
}

ct::TimeStamp parseTimeStamp(const JsonValue & json)
{
  ct::TimeStamp stamp;
  if (json["sec"].isNumber()) {
    stamp.sec = static_cast<std::int64_t>(json["sec"].asNumber());
  }
  if (json["nanosec"].isNumber()) {
    stamp.nsec = static_cast<std::int64_t>(json["nanosec"].asNumber());
  }
  return stamp;
}

ct::TrajectoryPoint parseTrajectoryPoint(const JsonValue & json)
{
  ct::TrajectoryPoint point;
  const auto & time_node = json["time_from_start"];
  if (time_node.isObject()) {
    point.time_from_start = parseDuration(time_node);
  }
  const auto & pose_node = json["pose"];
  point.pose = parsePose(pose_node);
  if (json["longitudinal_velocity_mps"].isNumber()) {
    point.longitudinal_velocity_mps = json["longitudinal_velocity_mps"].asNumber();
  }
  if (json["lateral_velocity_mps"].isNumber()) {
    point.lateral_velocity_mps = json["lateral_velocity_mps"].asNumber();
  }
  if (json["acceleration_mps2"].isNumber()) {
    point.acceleration_mps2 = json["acceleration_mps2"].asNumber();
  }
  if (json["heading_rate_rps"].isNumber()) {
    point.heading_rate_rps = json["heading_rate_rps"].asNumber();
  }
  if (json["front_wheel_angle_rad"].isNumber()) {
    point.front_wheel_angle_rad = json["front_wheel_angle_rad"].asNumber();
  }
  if (json["rear_wheel_angle_rad"].isNumber()) {
    point.rear_wheel_angle_rad = json["rear_wheel_angle_rad"].asNumber();
  }
  return point;
}

ct::Trajectory loadTrajectoryFromMessage(const JsonValue & message)
{
  ct::Trajectory trajectory;
  if (message["header"].isObject()) {
    const auto & header = message["header"];
    if (header["stamp"].isObject()) {
      trajectory.header.stamp = parseTimeStamp(header["stamp"]);
    }
    if (header["frame_id"].isString()) {
      trajectory.header.frame_id = header["frame_id"].asString();
    }
  }
  const auto & points = message["points"];
  if (!points.isArray()) {
    return trajectory;
  }
  trajectory.points.reserve(points.arraySize());
  for (std::size_t i = 0; i < points.arraySize(); ++i) {
    const auto & point_node = points[i];
    if (point_node.isObject()) {
      trajectory.points.push_back(parseTrajectoryPoint(point_node));
    }
  }
  return trajectory;
}

ct::KinematicState loadKinematicStateFromMessage(const JsonValue & message)
{
  ct::KinematicState state;
  if (message["header"].isObject()) {
    const auto & header = message["header"];
    if (header["stamp"].isObject()) {
      state.header.stamp = parseTimeStamp(header["stamp"]);
    }
    if (header["frame_id"].isString()) {
      state.header.frame_id = header["frame_id"].asString();
    }
  }
  state.pose = parsePose(message["pose"]);
  const auto & twist = message["twist"].isObject() ? message["twist"]["twist"] : JsonValue::nullValue();
  if (twist.isObject()) {
    const auto & linear = twist["linear"];
    const auto & angular = twist["angular"];
    if (linear.isObject()) {
      state.twist.linear_x = linear["x"].isNumber() ? linear["x"].asNumber() : 0.0;
      state.twist.linear_y = linear["y"].isNumber() ? linear["y"].asNumber() : 0.0;
      state.twist.linear_z = linear["z"].isNumber() ? linear["z"].asNumber() : 0.0;
    }
    if (angular.isObject()) {
      state.twist.angular_x = angular["x"].isNumber() ? angular["x"].asNumber() : 0.0;
      state.twist.angular_y = angular["y"].isNumber() ? angular["y"].asNumber() : 0.0;
      state.twist.angular_z = angular["z"].isNumber() ? angular["z"].asNumber() : 0.0;
    }
  }
  return state;
}

ct::Accel loadAccelerationFromMessage(const JsonValue & message)
{
  ct::Accel accel;
  const auto & accel_node = message["accel"].isObject() ? message["accel"]["accel"] : JsonValue::nullValue();
  if (accel_node.isObject()) {
    const auto & linear = accel_node["linear"];
    const auto & angular = accel_node["angular"];
    if (linear.isObject()) {
      accel.linear.x = linear["x"].isNumber() ? linear["x"].asNumber() : 0.0;
      accel.linear.y = linear["y"].isNumber() ? linear["y"].asNumber() : 0.0;
      accel.linear.z = linear["z"].isNumber() ? linear["z"].asNumber() : 0.0;
    }
    if (angular.isObject()) {
      accel.angular.x = angular["x"].isNumber() ? angular["x"].asNumber() : 0.0;
      accel.angular.y = angular["y"].isNumber() ? angular["y"].asNumber() : 0.0;
      accel.angular.z = angular["z"].isNumber() ? angular["z"].asNumber() : 0.0;
    }
  }
  return accel;
}

std::vector<ct::Point> parsePointArray(const JsonValue & array)
{
  std::vector<ct::Point> points;
  if (!array.isArray()) {
    return points;
  }
  points.reserve(array.arraySize());
  for (std::size_t i = 0; i < array.arraySize(); ++i) {
    const auto & element = array[i];
    if (element.isObject()) {
      points.push_back(parsePoint(element));
    }
  }
  return points;
}

std::vector<ct::PredictedPath> parsePredictedPaths(const JsonValue & array)
{
  std::vector<ct::PredictedPath> paths;
  if (!array.isArray()) {
    return paths;
  }
  for (std::size_t i = 0; i < array.arraySize(); ++i) {
    const auto & element = array[i];
    if (!element.isObject()) {
      continue;
    }
    ct::PredictedPath path;
    path.path = parsePointArray(element["path"]);
    if (element["time_step"].isNumber()) {
      path.time_step = element["time_step"].asNumber();
    }
    paths.push_back(path);
  }
  return paths;
}

ct::PredictedObject parsePredictedObject(const JsonValue & json)
{
  ct::PredictedObject object;
  if (json["uuid"].isString()) {
    object.uuid = json["uuid"].asString();
  }
  if (json["distance_along_traj"].isNumber()) {
    object.distance_along_traj = json["distance_along_traj"].asNumber();
  }
  if (json["lateral_distance"].isNumber()) {
    object.lateral_distance = json["lateral_distance"].asNumber();
  }
  if (json["velocity"].isNumber()) {
    object.velocity = json["velocity"].asNumber();
  }
  object.pose = parsePose(json);
  object.predicted_paths = parsePredictedPaths(json["predicted_paths"]);
  return object;
}

std::vector<ct::PredictedObject> loadPredictedObjectsFromMessage(const JsonValue & message)
{
  std::vector<ct::PredictedObject> objects;
  const auto & array = message["objects"];
  if (!array.isArray()) {
    return objects;
  }
  objects.reserve(array.arraySize());
  for (std::size_t i = 0; i < array.arraySize(); ++i) {
    const auto & element = array[i];
    if (element.isObject()) {
      objects.push_back(parsePredictedObject(element));
    }
  }
  return objects;
}

template <typename T>
struct Timestamped
{
  ct::TimeStamp stamp{};
  T data{};
};

ct::TimeStamp extractMessageStamp(const JsonValue & message)
{
  if (message["header"].isObject()) {
    const auto & header = message["header"];
    if (header["stamp"].isObject()) {
      return parseTimeStamp(header["stamp"]);
    }
  }
  return {};
}

bool isStampLessOrEqual(const ct::TimeStamp & lhs, const ct::TimeStamp & rhs)
{
  if (lhs.sec != rhs.sec) {
    return lhs.sec < rhs.sec;
  }
  return lhs.nsec <= rhs.nsec;
}

bool isSameStamp(const ct::TimeStamp & lhs, const ct::TimeStamp & rhs)
{
  return lhs.sec == rhs.sec && lhs.nsec == rhs.nsec;
}

std::vector<JsonValue> loadJsonMessages(const fs::path & path, size_t & count)
{
  std::ifstream ifs(path);
  std::vector<JsonValue> messages;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto doc = parseJsonLine(line);
    const auto & message = doc["message"];
    if (!message.isNull()) {
      messages.push_back(message);
    } else {
      messages.push_back(doc);
    }
  }
  count = messages.size();
  return messages;
}

std::vector<Timestamped<ct::Trajectory>> loadTrajectoryMessages(
  const fs::path & path, size_t & count)
{
  const auto docs = loadJsonMessages(path, count);
  std::vector<Timestamped<ct::Trajectory>> result;
  result.reserve(docs.size());
  for (const auto & message : docs) {
    Timestamped<ct::Trajectory> entry;
    entry.data = loadTrajectoryFromMessage(message);
    entry.stamp = extractMessageStamp(message);
    if (entry.stamp.sec == 0 && entry.stamp.nsec == 0) {
      entry.stamp = entry.data.header.stamp;
    }
    result.push_back(entry);
  }
  return result;
}

std::vector<Timestamped<ct::KinematicState>> loadKinematicStateMessages(
  const fs::path & path, size_t & count)
{
  const auto docs = loadJsonMessages(path, count);
  std::vector<Timestamped<ct::KinematicState>> result;
  result.reserve(docs.size());
  for (const auto & message : docs) {
    Timestamped<ct::KinematicState> entry;
    entry.data = loadKinematicStateFromMessage(message);
    entry.stamp = extractMessageStamp(message);
    if (entry.stamp.sec == 0 && entry.stamp.nsec == 0) {
      entry.stamp = entry.data.header.stamp;
    }
    result.push_back(entry);
  }
  return result;
}

std::vector<Timestamped<ct::Accel>> loadAccelerationMessages(
  const fs::path & path, size_t & count)
{
  const auto docs = loadJsonMessages(path, count);
  std::vector<Timestamped<ct::Accel>> result;
  result.reserve(docs.size());
  for (const auto & message : docs) {
    Timestamped<ct::Accel> entry;
    entry.data = loadAccelerationFromMessage(message);
    entry.stamp = extractMessageStamp(message);
    result.push_back(entry);
  }
  return result;
}

std::vector<Timestamped<std::vector<ct::PredictedObject>>> loadPredictedObjectsMessages(
  const fs::path & path, size_t & count)
{
  std::ifstream ifs(path);
  if (!ifs) {
    throw std::runtime_error("failed to open predicted objects file: " + path.string());
  }

  std::vector<Timestamped<std::vector<ct::PredictedObject>>> result;
  std::string line;
  count = 0;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto doc = parseJsonLine(line);
    JsonValue message = doc["message"];
    if (message.isNull()) {
      message = doc;
    }
    Timestamped<std::vector<ct::PredictedObject>> entry;
    entry.data = loadPredictedObjectsFromMessage(message);
    entry.stamp = extractMessageStamp(message);

    ct::TimeStamp outer_stamp{};
    if (doc["timestamp"].isNumber()) {
      const auto ns = static_cast<std::int64_t>(doc["timestamp"].asNumber());
      outer_stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
      outer_stamp.nsec = static_cast<std::int32_t>(ns % 1000000000LL);
    }

    if (entry.stamp.sec == 0 && entry.stamp.nsec == 0) {
      if (outer_stamp.sec != 0 || outer_stamp.nsec != 0) {
        entry.stamp = outer_stamp;
      } else {
        std::cerr << "warning: predicted objects message missing timestamp in " << path << "\n";
      }
    }

    result.push_back(entry);
    ++count;
  }
  return result;
}

ct::Trajectory loadTrajectoryFromFile(const fs::path & file_path)
{
  const auto maybe_message = loadLastMessage(file_path);
  if (!maybe_message) {
    throw std::runtime_error("failed to parse trajectory file: " + file_path.string());
  }
  return loadTrajectoryFromMessage(*maybe_message);
}

ct::KinematicState loadKinematicStateFromFile(const fs::path & file_path)
{
  const auto maybe_message = loadLastMessage(file_path);
  if (!maybe_message) {
    throw std::runtime_error("failed to parse kinematic state file: " + file_path.string());
  }
  return loadKinematicStateFromMessage(*maybe_message);
}

ct::Accel loadAccelerationFromFile(const fs::path & file_path)
{
  const auto maybe_message = loadLastMessage(file_path);
  if (!maybe_message) {
    return {};
  }
  return loadAccelerationFromMessage(*maybe_message);
}

std::vector<ct::PredictedObject> loadPredictedObjectsFromFile(const fs::path & file_path)
{
  const auto maybe_message = loadLastMessage(file_path);
  if (!maybe_message) {
    return {};
  }
  return loadPredictedObjectsFromMessage(*maybe_message);
}

double toScalar(const ct::Orientation & orientation)
{
  const double siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::optional<size_t> findStopIndex(const std::vector<ct::TrajectoryPoint> & points)
{
  const auto it = std::find_if(points.begin(), points.end(), [](const auto & point) {
    return point.longitudinal_velocity_mps < 0.01;
  });
  if (it == points.end()) {
    return std::nullopt;
  }
  return static_cast<size_t>(std::distance(points.begin(), it));
}

void appendTrajectoryJsonl(
  std::ofstream & ofs, const std::string & topic, const ct::TimeStamp & stamp,
  const ct::Trajectory & trajectory)
{
  if (!ofs) {
    throw std::runtime_error("output stream is not open");
  }
  const auto ns_timestamp = stamp.sec * 1000000000LL + stamp.nsec;
  const auto frame_id = trajectory.header.frame_id.empty() ? "map" : trajectory.header.frame_id;
  ofs << "{\"topic\": \"" << topic << "\", ";
  ofs << "\"timestamp\": " << ns_timestamp << ", ";
  ofs << "\"message\": {";
  ofs << "\"header\": {";
  ofs << "\"stamp\": {\"sec\": " << stamp.sec << ", \"nanosec\": " << stamp.nsec << "}, ";
  ofs << "\"frame_id\": \"" << frame_id << "\"}, ";
  ofs << "\"points\": [";
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    ofs << "{";
    ofs << "\"time_from_start\": {\"sec\": " << point.time_from_start.sec
        << ", \"nanosec\": " << point.time_from_start.nanosec << "}, ";
    ofs << "\"pose\": {\"position\": {";
    ofs << "\"x\": " << point.pose.position.x << ", ";
    ofs << "\"y\": " << point.pose.position.y << ", ";
    ofs << "\"z\": " << point.pose.position.z << "}, ";
    ofs << "\"orientation\": {";
    ofs << "\"x\": " << point.pose.orientation.x << ", ";
    ofs << "\"y\": " << point.pose.orientation.y << ", ";
    ofs << "\"z\": " << point.pose.orientation.z << ", ";
    ofs << "\"w\": " << point.pose.orientation.w << "}}, ";
    ofs << "\"longitudinal_velocity_mps\": " << point.longitudinal_velocity_mps << ", ";
    ofs << "\"lateral_velocity_mps\": " << point.lateral_velocity_mps << ", ";
    ofs << "\"acceleration_mps2\": " << point.acceleration_mps2 << ", ";
    ofs << "\"heading_rate_rps\": " << point.heading_rate_rps << ", ";
    ofs << "\"front_wheel_angle_rad\": " << point.front_wheel_angle_rad << ", ";
    ofs << "\"rear_wheel_angle_rad\": " << point.rear_wheel_angle_rad << "}";
    if (i + 1 < trajectory.points.size()) {
      ofs << ", ";
    }
  }
  ofs << "]}}\n";
}

struct ComparisonMetrics
{
  size_t ours_count{0};
  size_t answer_count{0};
  double max_pos_error{0.0};
  double max_yaw_error{0.0};
  double max_vel_error{0.0};
  double rmse_pos{0.0};
  double rmse_yaw{0.0};
  double rmse_vel{0.0};
  bool mismatch{false};
  size_t first_mismatch_point_index{std::numeric_limits<size_t>::max()};
  std::string mismatch_field;
};

ComparisonMetrics compareTrajectories(
  const ct::Trajectory & ours, const ct::Trajectory & answer, const double pos_tol,
  const double yaw_tol, const double vel_tol)
{
  ComparisonMetrics metrics;
  metrics.ours_count = ours.points.size();
  metrics.answer_count = answer.points.size();
  const size_t compare_count = std::min(metrics.ours_count, metrics.answer_count);
  double sum_sq_pos = 0.0;
  double sum_sq_yaw = 0.0;
  double sum_sq_vel = 0.0;
  for (size_t i = 0; i < compare_count; ++i) {
    const auto & ours_pt = ours.points[i];
    const auto & ans_pt = answer.points[i];
    const double dx = ours_pt.pose.position.x - ans_pt.pose.position.x;
    const double dy = ours_pt.pose.position.y - ans_pt.pose.position.y;
    const double pos_err = std::hypot(dx, dy);
    metrics.max_pos_error = std::max(metrics.max_pos_error, pos_err);
    sum_sq_pos += pos_err * pos_err;
    const double ours_yaw = normalizeAngle(toScalar(ours_pt.pose.orientation));
    const double ans_yaw = normalizeAngle(toScalar(ans_pt.pose.orientation));
    const double yaw_err = std::abs(normalizeAngle(ours_yaw - ans_yaw));
    metrics.max_yaw_error = std::max(metrics.max_yaw_error, yaw_err);
    sum_sq_yaw += yaw_err * yaw_err;
    const double vel_err = std::abs(ours_pt.longitudinal_velocity_mps - ans_pt.longitudinal_velocity_mps);
    metrics.max_vel_error = std::max(metrics.max_vel_error, vel_err);
    sum_sq_vel += vel_err * vel_err;
    if (!metrics.mismatch && (pos_err > pos_tol || yaw_err > yaw_tol || vel_err > vel_tol)) {
      metrics.mismatch = true;
      metrics.first_mismatch_point_index = i;
      if (pos_err > pos_tol) {
        metrics.mismatch_field = "position";
      } else if (yaw_err > yaw_tol) {
        metrics.mismatch_field = "yaw";
      } else {
        metrics.mismatch_field = "velocity";
      }
    }
  }
  if (compare_count > 0) {
    metrics.rmse_pos = std::sqrt(sum_sq_pos / compare_count);
    metrics.rmse_yaw = std::sqrt(sum_sq_yaw / compare_count);
    metrics.rmse_vel = std::sqrt(sum_sq_vel / compare_count);
  }
  return metrics;
}

std::optional<size_t> calcStopIndex(const ct::Trajectory & trajectory)
{
  return findStopIndex(trajectory.points);
}

double calcStopDistance(const ct::Trajectory & trajectory, const std::optional<size_t> idx)
{
  if (!idx) {
    return 0.0;
  }
  return acp::motion_utils::calcSignedArcLength(trajectory.points, 0, *idx);
}

constexpr double kStopVelocityThreshold = 0.1;
constexpr double kMonotonicTolerance = 1e-4;

std::optional<size_t> detectInitialStopFrame(const std::vector<double> & velocities)
{
  for (size_t i = 0; i < velocities.size(); ++i) {
    if (velocities[i] < kStopVelocityThreshold) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<size_t> detectResumeFrame(
  const std::vector<double> & velocities, const std::optional<size_t> & stop_frame)
{
  if (!stop_frame || *stop_frame + 1 >= velocities.size()) {
    return std::nullopt;
  }
  for (size_t i = *stop_frame + 1; i < velocities.size(); ++i) {
    if (velocities[i] >= kStopVelocityThreshold) {
      return i;
    }
  }
  return std::nullopt;
}

size_t calcStopDurationFrames(
  const std::optional<size_t> & stop_frame, const std::optional<size_t> & resume_frame,
  const size_t total_frames)
{
  if (!stop_frame || *stop_frame >= total_frames) {
    return 0;
  }
  const size_t end = resume_frame ? *resume_frame : total_frames;
  return end > *stop_frame ? end - *stop_frame : 0;
}

bool isVelocityRecoveryMonotonic(
  const std::vector<double> & velocities, const std::optional<size_t> & resume_frame)
{
  if (!resume_frame || *resume_frame + 1 >= velocities.size()) {
    return true;
  }
  for (size_t i = *resume_frame + 1; i < velocities.size(); ++i) {
    if (velocities[i] + kMonotonicTolerance < velocities[i - 1]) {
      return false;
    }
  }
  return true;
}

bool hasFinalGoalStop(const std::vector<double> & velocities)
{
  return !velocities.empty() && velocities.back() < kStopVelocityThreshold;
}

double extractLeadingVelocity(const ct::Trajectory & trajectory)
{
  return trajectory.points.empty() ? 0.0 : trajectory.points.front().longitudinal_velocity_mps;
}

struct FrameDiagnostics
{
  size_t frame_index{0};
  ct::TimeStamp stamp{};
  bool has_objects{false};
  size_t object_count{0};
  double planned_head_vel{0.0};
  double answer_head_vel{0.0};
  std::optional<size_t> planned_stop_idx;
  std::optional<size_t> answer_stop_idx;
  double planned_stop_dist{0.0};
  double answer_stop_dist{0.0};
};

int runScenario1(const fs::path & scenario_dir, const bool scenario_include_goal_stop)
{
  try {
    const fs::path kinematic_file = scenario_dir / "localization__kinematic_state.jsonl";
    const fs::path accel_file = scenario_dir / "localization__acceleration.jsonl";
    const fs::path objects_file = scenario_dir / "perception__object_recognition__objects.jsonl";
    const fs::path motion_trajectory_file = scenario_dir /
      "planning__scenario_planning__lane_driving__motion_planning__motion_velocity_planner__trajectory.jsonl";
    const fs::path answer_file = scenario_dir / "ANSWER_planning__scenario_planning__lane_driving__trajectory.jsonl";
    const fs::path output_file = scenario_dir / "planning__scenario_planning__lane_driving__trajectory.jsonl";

    if (!fs::exists(kinematic_file) || !fs::exists(motion_trajectory_file) || !fs::exists(answer_file)) {
      std::cerr << "Scenario1 files missing under " << scenario_dir << "\n";
      return 1;
    }

    size_t kin_count = 0;
    size_t accel_count = 0;
    size_t objects_count = 0;
    size_t motion_count = 0;
    size_t answer_count = 0;
    const auto kinematic_msgs = loadKinematicStateMessages(kinematic_file, kin_count);
    const auto accel_msgs = loadAccelerationMessages(accel_file, accel_count);
    const auto objects_msgs = loadPredictedObjectsMessages(objects_file, objects_count);
    const auto motion_msgs = loadTrajectoryMessages(motion_trajectory_file, motion_count);
    const auto answer_msgs = loadTrajectoryMessages(answer_file, answer_count);

    if (kinematic_msgs.empty() || motion_msgs.empty() || answer_msgs.empty()) {
      std::cerr << "Scenario1 lacks required message streams\n";
      return 1;
    }

    std::vector<std::pair<std::string, size_t>> message_counts{
      {"localization__kinematic_state", kin_count},
      {"localization__acceleration", accel_count},
      {"perception__object_recognition__objects", objects_count},
      {"planning__scenario_planning__lane_driving__motion_planning__motion_velocity_planner__trajectory",
       motion_count},
      {"ANSWER_planning__scenario_planning__lane_driving__trajectory", answer_count},
    };

    std::ofstream output_stream(output_file);
    if (!output_stream) {
      throw std::runtime_error("failed to open output file: " + output_file.string());
    }

    const std::string trajectory_topic = "/planning/scenario_planning/lane_driving/trajectory";

    acp::LongitudinalInfo info;
    info.safe_distance_margin = 5.0;
    info.terminal_safe_distance_margin = 3.0;
    info.hold_stop_distance_threshold = 0.3;
    info.hold_stop_velocity_threshold = 0.01;
    acp::BehaviorDeterminationParam behavior_param;
    behavior_param.max_lat_margin_for_stop = 0.8;
    behavior_param.max_lat_margin_for_slow_down = 1.2;
    behavior_param.stop_distance_threshold = 8.0;
    behavior_param.slow_down_distance_margin = 5.0;
    acp::ObstacleCruisePlannerNode node(info, behavior_param);

    size_t kin_idx = 0;
    size_t accel_idx = 0;
    size_t obj_idx = 0;
    std::optional<ct::KinematicState> last_kin;
    std::optional<ct::Accel> last_acc;
    std::vector<ct::PredictedObject> last_objects;
    std::optional<ct::TimeStamp> last_objects_stamp;
    if (!kinematic_msgs.empty()) {
      last_kin = kinematic_msgs.front().data;
    }

    size_t output_frames = 0;
    size_t frames_compared = 0;
    double global_max_pos_error = 0.0;
    double global_max_yaw_error = 0.0;
    double global_max_vel_error = 0.0;
    double global_rmse_pos = 0.0;
    double global_rmse_yaw = 0.0;
    double global_rmse_vel = 0.0;
    size_t answer_cursor = 0;

    bool mismatch_found = false;
    size_t mismatch_frame_idx = 0;
    size_t mismatch_point_idx = 0;
    ct::TimeStamp mismatch_stamp{};
    std::string mismatch_field;
    std::optional<ct::Trajectory> mismatch_planned;
    std::optional<ct::Trajectory> mismatch_answer;

    bool stop_mismatch_found = false;
    size_t stop_mismatch_frame_idx = 0;
    ct::TimeStamp stop_mismatch_stamp{};
    std::optional<size_t> stop_idx_ours;
    std::optional<size_t> stop_idx_answer;
    double stop_dist_ours = 0.0;
    double stop_dist_answer = 0.0;

    const double pos_tol = 1e-2;
    const double yaw_tol = 1e-3;
    const double vel_tol = 1e-3;
    const double stop_dist_tol = 1e-2;

    for (size_t motion_idx = 0; motion_idx < motion_msgs.size(); ++motion_idx) {
      const auto & frame = motion_msgs[motion_idx];
      const bool include_goal_stop =
        scenario_include_goal_stop && (motion_idx + 1 == motion_msgs.size());
      while (kin_idx < kinematic_msgs.size() &&
             isStampLessOrEqual(kinematic_msgs[kin_idx].stamp, frame.stamp)) {
        last_kin = kinematic_msgs[kin_idx].data;
        ++kin_idx;
      }
      while (accel_idx < accel_msgs.size() &&
             isStampLessOrEqual(accel_msgs[accel_idx].stamp, frame.stamp)) {
        last_acc = accel_msgs[accel_idx].data;
        ++accel_idx;
      }
      while (obj_idx < objects_msgs.size() &&
             isStampLessOrEqual(objects_msgs[obj_idx].stamp, frame.stamp)) {
        last_objects = objects_msgs[obj_idx].data;
        last_objects_stamp = objects_msgs[obj_idx].stamp;
        ++obj_idx;
      }
      if (!last_kin) {
        continue;
      }

      const double ego_velocity = last_kin->twist.linear_x;
      const double ego_acceleration = last_acc ? last_acc->linear.x : 0.0;
      size_t matched_answer_idx = std::numeric_limits<size_t>::max();
      while (answer_cursor < answer_msgs.size() &&
             isStampLessOrEqual(answer_msgs[answer_cursor].stamp, frame.stamp)) {
        if (isSameStamp(answer_msgs[answer_cursor].stamp, frame.stamp)) {
          matched_answer_idx = answer_cursor;
          ++answer_cursor;
          break;
        }
        ++answer_cursor;
      }

      if (matched_answer_idx == std::numeric_limits<size_t>::max()) {
        continue;
      }

      const auto planned_points =
        node.planTrajectory(
          frame.data, ego_velocity, ego_acceleration, last_objects, last_objects_stamp,
          include_goal_stop);
      ct::Trajectory planned;
      planned.points = planned_points;
      planned.header.frame_id =
        frame.data.header.frame_id.empty() ? "map" : frame.data.header.frame_id;
      planned.header.stamp = frame.stamp;

      appendTrajectoryJsonl(output_stream, trajectory_topic, frame.stamp, planned);

      const auto & answer_traj = answer_msgs[matched_answer_idx].data;
      const auto metrics = compareTrajectories(planned, answer_traj, pos_tol, yaw_tol, vel_tol);
      global_max_pos_error = std::max(global_max_pos_error, metrics.max_pos_error);
      global_max_yaw_error = std::max(global_max_yaw_error, metrics.max_yaw_error);
      global_max_vel_error = std::max(global_max_vel_error, metrics.max_vel_error);
      global_rmse_pos = std::max(global_rmse_pos, metrics.rmse_pos);
      global_rmse_yaw = std::max(global_rmse_yaw, metrics.rmse_yaw);
      global_rmse_vel = std::max(global_rmse_vel, metrics.rmse_vel);
      if (!mismatch_found && metrics.mismatch) {
        mismatch_found = true;
        mismatch_frame_idx = output_frames;
        mismatch_point_idx = metrics.first_mismatch_point_index;
        mismatch_stamp = frame.stamp;
        mismatch_field = metrics.mismatch_field;
        mismatch_planned = planned;
        mismatch_answer = answer_traj;
      }
      const auto ours_stop_idx = calcStopIndex(planned);
      const auto answer_stop_idx = calcStopIndex(answer_traj);
      const double ours_stop_dist = calcStopDistance(planned, ours_stop_idx);
      const double answer_stop_dist = calcStopDistance(answer_traj, answer_stop_idx);
      const double stop_delta = std::abs(ours_stop_dist - answer_stop_dist);
      if (!stop_mismatch_found &&
          (ours_stop_idx != answer_stop_idx || stop_delta > stop_dist_tol)) {
        stop_mismatch_found = true;
        stop_mismatch_frame_idx = output_frames;
        stop_mismatch_stamp = frame.stamp;
        stop_idx_ours = ours_stop_idx;
        stop_idx_answer = answer_stop_idx;
        stop_dist_ours = ours_stop_dist;
        stop_dist_answer = answer_stop_dist;
      }
      ++frames_compared;
      ++output_frames;
    }

    output_stream.flush();

    std::cout << "Scenario1 streaming comparison summary:\n";
    std::cout << "  messages loaded:\n";
    for (const auto & [name, count] : message_counts) {
      std::cout << "    " << name << ": " << count << "\n";
    }
    std::cout << "  output trajectory file: " << output_file << " (" << output_frames << " frames)\n";
    std::cout << "  frames compared against answer: " << frames_compared << "\n";
    std::cout << "  max errors (pos/yaw/vel): "
              << global_max_pos_error << " / " << global_max_yaw_error << " / "
              << global_max_vel_error << "\n";
    std::cout << "  max RMSE (pos/yaw/vel): "
              << global_rmse_pos << " / " << global_rmse_yaw << " / " << global_rmse_vel << "\n";
    if (mismatch_found && mismatch_planned && mismatch_answer) {
      std::cout << "  first mismatch frame=" << mismatch_frame_idx << " timestamp="
                << mismatch_stamp.sec << "." << mismatch_stamp.nsec << " field=" << mismatch_field
                << " point=" << mismatch_point_idx << "\n";
      if (mismatch_point_idx < mismatch_planned->points.size() &&
          mismatch_point_idx < mismatch_answer->points.size()) {
        const auto & ours_pt = mismatch_planned->points[mismatch_point_idx];
        const auto & ans_pt = mismatch_answer->points[mismatch_point_idx];
        if (mismatch_field == "position") {
          const double dx = ours_pt.pose.position.x - ans_pt.pose.position.x;
          const double dy = ours_pt.pose.position.y - ans_pt.pose.position.y;
          std::cout << "    ours_pos=(" << ours_pt.pose.position.x << ", " << ours_pt.pose.position.y
                    << ") answer_pos=(" << ans_pt.pose.position.x << ", " << ans_pt.pose.position.y
                    << ") delta=" << std::hypot(dx, dy) << "\n";
        } else if (mismatch_field == "yaw") {
          const double ours_yaw = normalizeAngle(toScalar(ours_pt.pose.orientation));
          const double ans_yaw = normalizeAngle(toScalar(ans_pt.pose.orientation));
          std::cout << "    ours_yaw=" << ours_yaw << " answer_yaw=" << ans_yaw << " delta="
                    << std::abs(normalizeAngle(ours_yaw - ans_yaw)) << "\n";
        } else if (mismatch_field == "velocity") {
          std::cout << "    ours_vel=" << ours_pt.longitudinal_velocity_mps
                    << " answer_vel=" << ans_pt.longitudinal_velocity_mps << "\n";
        }
      }
    } else {
      std::cout << "  trajectories matched within tolerance\n";
    }
    if (stop_mismatch_found) {
      std::cout << "  stop mismatch at frame=" << stop_mismatch_frame_idx << " timestamp="
                << stop_mismatch_stamp.sec << "." << stop_mismatch_stamp.nsec << "\n";
      std::cout << "    ours_stop_index="
                << (stop_idx_ours ? std::to_string(*stop_idx_ours) : "n/a")
                << " answer_stop_index="
                << (stop_idx_answer ? std::to_string(*stop_idx_answer) : "n/a") << "\n";
      std::cout << "    stop distance ours=" << stop_dist_ours << " answer=" << stop_dist_answer
                << "\n";
    } else {
      std::cout << "  stop metrics matched within tolerance\n";
    }
    return mismatch_found ? 1 : 0;
  } catch (const std::exception & ex) {
    std::cerr << "Scenario1 failed: " << ex.what() << "\n";
    return 1;
  }
}

int runScenario3(const fs::path & scenario_dir)
{
  try {
    const fs::path kinematic_file = scenario_dir / "localization__kinematic_state.jsonl";
    const fs::path accel_file = scenario_dir / "localization__acceleration.jsonl";
    const fs::path objects_file = scenario_dir / "perception__object_recognition__objects.jsonl";
    const fs::path motion_trajectory_file = scenario_dir /
      "planning__scenario_planning__lane_driving__motion_planning__motion_velocity_planner__trajectory.jsonl";
    const fs::path answer_file = scenario_dir /
      "ANSWER_planning__scenario_planning__lane_driving__trajectory.jsonl";
    const fs::path output_file = scenario_dir / "planning__scenario_planning__lane_driving__trajectory.jsonl";

    if (!fs::exists(kinematic_file) || !fs::exists(motion_trajectory_file) || !fs::exists(answer_file)) {
      std::cerr << "Scenario3 files missing under " << scenario_dir << "\n";
      return 1;
    }

    size_t kin_count = 0;
    size_t accel_count = 0;
    size_t objects_count = 0;
    size_t motion_count = 0;
    size_t answer_count = 0;
    const auto kinematic_msgs = loadKinematicStateMessages(kinematic_file, kin_count);
    const auto accel_msgs = loadAccelerationMessages(accel_file, accel_count);
    const auto objects_msgs = loadPredictedObjectsMessages(objects_file, objects_count);
    const auto motion_msgs = loadTrajectoryMessages(motion_trajectory_file, motion_count);
    const auto answer_msgs = loadTrajectoryMessages(answer_file, answer_count);

    if (kinematic_msgs.empty() || motion_msgs.empty() || answer_msgs.empty()) {
      std::cerr << "Scenario3 lacks required message streams\n";
      return 1;
    }

    std::vector<std::pair<std::string, size_t>> message_counts{
      {"localization__kinematic_state", kin_count},
      {"localization__acceleration", accel_count},
      {"perception__object_recognition__objects", objects_count},
      {"planning__scenario_planning__lane_driving__motion_planning__motion_velocity_planner__trajectory",
       motion_count},
      {"ANSWER_planning__scenario_planning__lane_driving__trajectory", answer_count},
    };

    std::ofstream output_stream(output_file);
    if (!output_stream) {
      throw std::runtime_error("failed to open output file: " + output_file.string());
    }

    const std::string trajectory_topic = "/planning/scenario_planning/lane_driving/trajectory";

    acp::LongitudinalInfo info;
    info.safe_distance_margin = 1.0;
    info.terminal_safe_distance_margin = 0.0;
    info.hold_stop_distance_threshold = 0.5;
    info.hold_stop_velocity_threshold = 0.2;
    acp::BehaviorDeterminationParam behavior_param;
    behavior_param.max_lat_margin_for_stop = 0.8;
    behavior_param.max_lat_margin_for_slow_down = 1.2;
    behavior_param.stop_distance_threshold = 120.0;
    behavior_param.slow_down_distance_margin = 5.0;
    behavior_param.obstacle_velocity_threshold_from_stop_to_cruise = 3.5;
    behavior_param.stop_obstacle_hold_time_threshold = 1.0;
    acp::ObstacleCruisePlannerNode node(info, behavior_param);

    size_t kin_idx = 0;
    size_t accel_idx = 0;
    size_t obj_idx = 0;
    std::optional<ct::KinematicState> last_kin;
    std::optional<ct::Accel> last_acc;
    std::vector<ct::PredictedObject> last_objects;
    std::optional<ct::TimeStamp> last_objects_stamp;
    if (!kinematic_msgs.empty()) {
      last_kin = kinematic_msgs.front().data;
    }

    size_t output_frames = 0;
    size_t frames_compared = 0;
    double global_max_pos_error = 0.0;
    double global_max_yaw_error = 0.0;
    double global_max_vel_error = 0.0;
    double global_rmse_pos = 0.0;
    double global_rmse_yaw = 0.0;
    double global_rmse_vel = 0.0;
    size_t answer_cursor = 0;

    bool mismatch_found = false;
    size_t mismatch_frame_idx = 0;
    size_t mismatch_point_idx = 0;
    ct::TimeStamp mismatch_stamp{};
    std::string mismatch_field;
    std::optional<ct::Trajectory> mismatch_planned;
    std::optional<ct::Trajectory> mismatch_answer;

    const double pos_tol = 1e-2;
    const double yaw_tol = 1e-3;
    const double vel_tol = 1e-3;

    std::vector<double> ours_head_vels;
    std::vector<double> answer_head_vels;

    for (size_t motion_idx = 0; motion_idx < motion_msgs.size(); ++motion_idx) {
      const auto & frame = motion_msgs[motion_idx];
      const bool include_goal_stop = true;
      while (kin_idx < kinematic_msgs.size() &&
             isStampLessOrEqual(kinematic_msgs[kin_idx].stamp, frame.stamp)) {
        last_kin = kinematic_msgs[kin_idx].data;
        ++kin_idx;
      }
      while (accel_idx < accel_msgs.size() &&
             isStampLessOrEqual(accel_msgs[accel_idx].stamp, frame.stamp)) {
        last_acc = accel_msgs[accel_idx].data;
        ++accel_idx;
      }
      while (obj_idx < objects_msgs.size() &&
             isStampLessOrEqual(objects_msgs[obj_idx].stamp, frame.stamp)) {
        last_objects = objects_msgs[obj_idx].data;
        last_objects_stamp = objects_msgs[obj_idx].stamp;
        ++obj_idx;
      }
      if (!last_kin) {
        continue;
      }

      const double ego_velocity = last_kin->twist.linear_x;
      const double ego_acceleration = last_acc ? last_acc->linear.x : 0.0;
      size_t matched_answer_idx = std::numeric_limits<size_t>::max();
      while (answer_cursor < answer_msgs.size() &&
             isStampLessOrEqual(answer_msgs[answer_cursor].stamp, frame.stamp)) {
        if (isSameStamp(answer_msgs[answer_cursor].stamp, frame.stamp)) {
          matched_answer_idx = answer_cursor;
          ++answer_cursor;
          break;
        }
        ++answer_cursor;
      }

      if (matched_answer_idx == std::numeric_limits<size_t>::max()) {
        continue;
      }

      const auto planned_points = node.planTrajectory(
        frame.data, ego_velocity, ego_acceleration, last_objects, last_objects_stamp,
        include_goal_stop);
      ct::Trajectory planned;
      planned.points = planned_points;
      planned.header.frame_id =
        frame.data.header.frame_id.empty() ? "map" : frame.data.header.frame_id;
      planned.header.stamp = frame.stamp;

      appendTrajectoryJsonl(output_stream, trajectory_topic, frame.stamp, planned);

      const auto & answer_traj = answer_msgs[matched_answer_idx].data;
      const auto metrics = compareTrajectories(planned, answer_traj, pos_tol, yaw_tol, vel_tol);
      global_max_pos_error = std::max(global_max_pos_error, metrics.max_pos_error);
      global_max_yaw_error = std::max(global_max_yaw_error, metrics.max_yaw_error);
      global_max_vel_error = std::max(global_max_vel_error, metrics.max_vel_error);
      global_rmse_pos = std::max(global_rmse_pos, metrics.rmse_pos);
      global_rmse_yaw = std::max(global_rmse_yaw, metrics.rmse_yaw);
      global_rmse_vel = std::max(global_rmse_vel, metrics.rmse_vel);
      if (!mismatch_found && metrics.mismatch) {
        mismatch_found = true;
        mismatch_frame_idx = output_frames;
        mismatch_point_idx = metrics.first_mismatch_point_index;
        mismatch_stamp = frame.stamp;
        mismatch_field = metrics.mismatch_field;
        mismatch_planned = planned;
        mismatch_answer = answer_traj;
      }

      const double ours_head_vel =
        planned.points.empty() ? 0.0 : planned.points.front().longitudinal_velocity_mps;
      const double answer_head_vel =
        answer_traj.points.empty() ? 0.0 : answer_traj.points.front().longitudinal_velocity_mps;
      ours_head_vels.push_back(ours_head_vel);
      answer_head_vels.push_back(answer_head_vel);

      ++frames_compared;
      ++output_frames;
    }

    output_stream.flush();

    const size_t total_frames = answer_head_vels.size();
    const auto answer_stop_frame = detectInitialStopFrame(answer_head_vels);
    const auto answer_resume_frame =
      detectResumeFrame(answer_head_vels, answer_stop_frame);
    const auto ours_stop_frame = detectInitialStopFrame(ours_head_vels);
    const auto ours_resume_frame = detectResumeFrame(ours_head_vels, ours_stop_frame);
    const size_t answer_stop_duration =
      calcStopDurationFrames(answer_stop_frame, answer_resume_frame, total_frames);
    const size_t ours_stop_duration =
      calcStopDurationFrames(ours_stop_frame, ours_resume_frame, total_frames);
    const bool answer_monotonic =
      isVelocityRecoveryMonotonic(answer_head_vels, answer_resume_frame);
    const bool ours_monotonic =
      isVelocityRecoveryMonotonic(ours_head_vels, ours_resume_frame);
    const bool answer_final_stop = hasFinalGoalStop(answer_head_vels);
    const bool ours_final_stop = hasFinalGoalStop(ours_head_vels);

    std::string phaseA_range = "n/a";
    std::string phaseB_range = "n/a";
    std::string phaseC_range = "n/a";
    if (total_frames > 0) {
      if (!answer_stop_frame) {
        phaseA_range = "0-" + std::to_string(total_frames - 1);
      } else {
        const size_t phaseA_end = *answer_stop_frame == 0 ? 0 : *answer_stop_frame - 1;
        phaseA_range = "0-" + std::to_string(phaseA_end);
        const size_t phaseB_end =
          answer_resume_frame ? std::max<size_t>(0, *answer_resume_frame - 1) : total_frames - 1;
        phaseB_range = std::to_string(*answer_stop_frame) + "-" + std::to_string(phaseB_end);
        if (answer_resume_frame && *answer_resume_frame < total_frames) {
          phaseC_range = std::to_string(*answer_resume_frame) + "-" +
                         std::to_string(total_frames - 1);
        }
      }
    }

    const auto formatFrameIdx = [](const std::optional<size_t> & idx) {
      return idx ? std::to_string(*idx) : "n/a";
    };
    const bool stop_frame_mismatch = ours_stop_frame != answer_stop_frame;
    const bool resume_frame_mismatch = ours_resume_frame != answer_resume_frame;
    const bool stop_duration_mismatch = ours_stop_duration != answer_stop_duration;
    const bool monotonicity_mismatch = ours_monotonic != answer_monotonic;
    const bool final_stop_mismatch = ours_final_stop != answer_final_stop;
    mismatch_found = mismatch_found || stop_frame_mismatch || resume_frame_mismatch ||
                     stop_duration_mismatch || monotonicity_mismatch || final_stop_mismatch;

    std::cout << "Scenario3 stop/resume comparison summary:\n";
    std::cout << "  messages loaded:\n";
    for (const auto & [name, count] : message_counts) {
      std::cout << "    " << name << ": " << count << "\n";
    }
    std::cout << "  output trajectory file: " << output_file << " (" << output_frames << " frames)\n";
    std::cout << "  frames compared against answer: " << frames_compared << "\n";
    std::cout << "  max errors (pos/yaw/vel): "
              << global_max_pos_error << " / " << global_max_yaw_error << " / "
              << global_max_vel_error << "\n";
    std::cout << "  max RMSE (pos/yaw/vel): "
              << global_rmse_pos << " / " << global_rmse_yaw << " / " << global_rmse_vel << "\n";
    std::cout << "  Phase A range: " << phaseA_range << "\n";
    std::cout << "  Phase B range: " << phaseB_range << "\n";
    std::cout << "  Phase C range: " << phaseC_range << "\n";
    std::cout << "  stop/resume frames (ours vs answer): initial_stop="
              << formatFrameIdx(ours_stop_frame) << "/" << formatFrameIdx(answer_stop_frame)
              << ", stop_duration=" << ours_stop_duration << "/" << answer_stop_duration
              << ", resume=" << formatFrameIdx(ours_resume_frame) << "/"
              << formatFrameIdx(answer_resume_frame) << "\n";
    std::cout << "  velocity recovery monotonic (ours/answer): "
              << (ours_monotonic ? "yes" : "no") << "/"
              << (answer_monotonic ? "yes" : "no") << "\n";
    std::cout << "  final goal stop (ours/answer): "
              << (ours_final_stop ? "yes" : "no") << "/"
              << (answer_final_stop ? "yes" : "no") << "\n";
    if (mismatch_found && mismatch_planned && mismatch_answer) {
      std::cout << "  first trajectory mismatch frame=" << mismatch_frame_idx << " timestamp="
                << mismatch_stamp.sec << "." << mismatch_stamp.nsec << " field=" << mismatch_field
                << " point=" << mismatch_point_idx << "\n";
    }
    return mismatch_found ? 1 : 0;
  } catch (const std::exception & ex) {
    std::cerr << "Scenario3 failed: " << ex.what() << "\n";
    return 1;
  }
}

void runDefaultMode(const std::map<std::string, std::string> & config)
{
  auto readDouble = [&](const std::string & key, const double fallback) {
    if (config.find(key) == config.end()) {
      return fallback;
    }
    try {
      return std::stod(config.at(key));
    } catch (...) {
      return fallback;
    }
  };

  acp::LongitudinalInfo info;
  info.safe_distance_margin = readDouble("safe_distance_margin", 1.0);
  info.terminal_safe_distance_margin = readDouble("safe_distance_margin", 1.0);
  info.hold_stop_distance_threshold = readDouble("min_behavior_stop_margin", 0.5);
  info.hold_stop_velocity_threshold = readDouble("obstacle_velocity_threshold_from_stop_to_cruise", 0.2);

  acp::BehaviorDeterminationParam behavior_param;
  behavior_param.max_lat_margin_for_stop = readDouble("max_lat_margin_for_stop", 0.8);
  behavior_param.max_lat_margin_for_slow_down = readDouble("max_lat_margin_for_slow_down", 1.2);
  behavior_param.stop_distance_threshold = readDouble("stop_distance_threshold", 8.0);
  behavior_param.slow_down_distance_margin = readDouble("slow_down_distance_margin", 5.0);

  acp::ObstacleCruisePlannerNode node(info, behavior_param);

  const auto runScenario = [&](const std::string & name, const ct::Trajectory & trajectory,
                               const std::vector<ct::PredictedObject> & predicted_objects) {
    const auto planned = node.planTrajectory(
      trajectory, 2.0, 0.0, predicted_objects, std::nullopt);
    const auto zero_it = std::find_if(
      planned.begin(), planned.end(), [](const auto & point) { return point.longitudinal_velocity_mps < 0.01; });
    const std::optional<size_t> zero_idx = zero_it == planned.end()
                                              ? std::optional<size_t>{}
                                              : std::optional<size_t>{static_cast<size_t>(zero_it - planned.begin())};
    const double stop_distance = zero_idx
                                   ? acp::motion_utils::calcSignedArcLength(planned, 0, *zero_idx)
                                   : 0.0;
    std::cout << name << " → size=" << planned.size();
    if (zero_idx) {
      std::cout << ", stop_index=" << *zero_idx << ", stop_distance=" << stop_distance;
    } else {
      std::cout << ", stop_index=n/a";
    }
    std::cout << ", final_vel="
              << (planned.empty() ? 0.0 : planned.back().longitudinal_velocity_mps)
              << ", obstacles=" << predicted_objects.size() << "\n";
  };

  const auto straight = createStraightTrajectory(20.0, 20);
  runScenario("Scenario Straight", straight, std::vector<ct::PredictedObject>{});

  const auto left_turn = createLeftTurnTrajectory(10.0, 20);
  runScenario("Scenario Left Turn", left_turn, std::vector<ct::PredictedObject>{});

  const auto obstacle = createStaticObstacle(12.0);
  runScenario("Scenario Obstacle Present", straight, std::vector<ct::PredictedObject>{obstacle});
  runScenario("Scenario Obstacle Cleared", straight, std::vector<ct::PredictedObject>{});
}
}  // namespace

int main(int argc, char ** argv)
{
  enum class RunMode { Normal, Scenario1, Scenario3 };
  RunMode mode = RunMode::Normal;
  std::string config_path = "config/default.yaml";
  fs::path scenario_dir;
  bool scenario1_include_goal_stop = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scenario1-include-goal-stop") {
      scenario1_include_goal_stop = true;
      continue;
    }
    if (arg == "--scenario1") {
      mode = RunMode::Scenario1;
      if (i + 1 < argc) {
        scenario_dir = argv[++i];
        continue;
      }
      std::cerr << "--scenario1 requires a directory argument\n";
      return 1;
    }
    if (arg == "--scenario3") {
      mode = RunMode::Scenario3;
      if (i + 1 < argc) {
        scenario_dir = argv[++i];
        continue;
      }
      std::cerr << "--scenario3 requires a directory argument\n";
      return 1;
    }
    config_path = arg;
  }

  if (mode == RunMode::Scenario1) {
    if (scenario_dir.empty()) {
      std::cerr << "Scenario directory is required\n";
      return 1;
    }
    return runScenario1(scenario_dir, scenario1_include_goal_stop);
  }
  if (mode == RunMode::Scenario3) {
    if (scenario_dir.empty()) {
      std::cerr << "Scenario directory is required\n";
      return 1;
    }
    return runScenario3(scenario_dir);
  }

  std::map<std::string, std::string> config;
  try {
    config = loadConfig(config_path);
    std::cout << "Config loaded (" << config.size() << " entries):\n";
    for (const auto & [key, value] : config) {
      std::cout << "  " << key << " = " << value << "\n";
    }
  } catch (const std::exception & ex) {
    std::cerr << "Failed to initialize: " << ex.what() << "\n";
    return 1;
  }

  runDefaultMode(config);
  return 0;
}
