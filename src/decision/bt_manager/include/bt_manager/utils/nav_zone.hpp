#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
namespace Sentry_BT {
struct Point2D
{
  double x;
  double y;
  double yaw;
  Point2D() : x(0.0), y(0.0), yaw(0.0) {}
  Point2D(double x_, double y_, double yaw_ = 0.0) : x(x_), y(y_), yaw(yaw_) {}
  Point2D operator-(const Point2D & other) const
  {
    return Point2D(x - other.x, y - other.y, yaw - other.yaw);
  }
  Point2D operator+(const Point2D & other) const
  {
    return Point2D(x + other.x, y + other.y, yaw + other.yaw);
  }
};

struct TunnelRecoveryConfig
{
  float tunnel_pass_yaw_target_rad;  // Desired yaw when passing this tunnel, in radians
  Point2D recovery_point;            // Recovery/retreat point for this tunnel
  Point2D forward_point;             // Forward attempt point for this tunnel
  float recovery_vx;                 // Recovery linear velocity x in map frame
  float recovery_vy;                 // Recovery linear velocity y in map frame

  TunnelRecoveryConfig()
  : tunnel_pass_yaw_target_rad(0.0f), recovery_point(), forward_point(), recovery_vx(0.0f),
    recovery_vy(0.0f)
  {
  }

  TunnelRecoveryConfig(
    float tunnel_pass_yaw_target_rad_,
    const Point2D & recovery_point_,
    const Point2D & forward_point_,
    float recovery_vx_,
    float recovery_vy_)
  : tunnel_pass_yaw_target_rad(tunnel_pass_yaw_target_rad_), recovery_point(recovery_point_),
    forward_point(forward_point_), recovery_vx(recovery_vx_), recovery_vy(recovery_vy_)
  {
  }
};

struct GimbalPatrolPoint
{
  float yaw_lower_bound_deg;  // 云台巡逻点的偏航角下界，单位为度
  float yaw_upper_bound_deg;  // 云台巡逻点的偏航角上界
  bool pitch_up;              // 是否需要抬头

  GimbalPatrolPoint() : yaw_lower_bound_deg(0.0), yaw_upper_bound_deg(0.0), pitch_up(false) {}
  GimbalPatrolPoint(float yaw_lower_bound_deg_, float yaw_upper_bound_deg_, bool pitch_up_)
  : yaw_lower_bound_deg(yaw_lower_bound_deg_), yaw_upper_bound_deg(yaw_upper_bound_deg_),
    pitch_up(pitch_up_)
  {
  }
};
struct Area_Square
{
  Point2D top_left;
  Point2D bottom_right;

  Area_Square() : top_left(), bottom_right() {}
  Area_Square(const Point2D & top_left_, const Point2D & bottom_right_)
  : top_left(top_left_), bottom_right(bottom_right_)
  {
  }

  bool contains(const Point2D & point) const
  {
    double min_x = std::min(top_left.x, bottom_right.x);
    double max_x = std::max(top_left.x, bottom_right.x);
    double min_y = std::min(top_left.y, bottom_right.y);
    double max_y = std::max(top_left.y, bottom_right.y);
    return (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y);
  }
};

struct Area_Circle
{
  Point2D center;
  double radius;

  Area_Circle() : center(), radius(0.0) {}
  Area_Circle(const Point2D & center_, double radius_) : center(center_), radius(radius_) {}

  bool contains(const Point2D & point) const
  {
    double dx = point.x - center.x;
    double dy = point.y - center.y;
    return (dx * dx + dy * dy) <= (radius * radius);
  }
};

template <std::size_t N, typename T = Point2D> struct AreaPolygon
{
  static_assert(N >= 3, "AreaPolygon requires at least 3 vertices");

  std::array<T, N> vertices;

  AreaPolygon() = default;

  template <typename... Args> explicit AreaPolygon(Args... args) : vertices{args...}
  {
    static_assert(sizeof...(args) == N, "Number of arguments must match template parameter N");
  }

  bool contains(const T & pt) const
  {
    constexpr double eps = 1e-9;
    bool inside = false;

    for (std::size_t i = 0, j = N - 1; i < N; j = i++) {
      const T & a = vertices[j];
      const T & b = vertices[i];

      const double min_y = std::min(a.y, b.y);
      const double max_y = std::max(a.y, b.y);

      // Fast y-range rejection before expensive intersection math.
      if (pt.y < (min_y - eps) || pt.y > (max_y + eps)) {
        continue;
      }

      const double abx = b.x - a.x;
      const double aby = b.y - a.y;
      const double apx = pt.x - a.x;
      const double apy = pt.y - a.y;

      // Boundary-inclusive check for colinear points on edges.
      const double cross = abx * apy - aby * apx;
      if (std::fabs(cross) <= eps) {
        const double dot = apx * abx + apy * aby;
        const double len2 = abx * abx + aby * aby;
        if (dot >= -eps && dot <= (len2 + eps)) {
          return true;
        }
      }

      // Vertex-safe crossing rule with strict x-intersection test.
      const bool intersects = ((b.y > pt.y) != (a.y > pt.y));
      if (intersects) {
        const double denom = a.y - b.y;
        if (std::fabs(denom) <= eps) {
          continue;
        }
        const double x_intersection = (a.x - b.x) * (pt.y - b.y) / denom + b.x;
        if (pt.x < x_intersection - eps) {
          inside = !inside;
        }
      }
    }

    return inside;
  }
};

struct PatrolPoint
{
  Point2D position;
  int duration_ms;  // 在该巡逻点停留的时间，单位为毫秒

  PatrolPoint() : position(), duration_ms(0) {}
  PatrolPoint(const Point2D & position_, int duration_ms_) : position(position_), duration_ms(duration_ms_)
  {
  }
};

typedef enum _TacticalMode
{
  OFFENSIVE = 0,
  DEFENSIVE = 1,
  BALANCED = 2,
} TacticalMode;

typedef enum _LifterPos
{
  TOP = 0,     // 云台顶部
  BOTTOM = 1,  // 云台底部
  MIDDLE = 2   // 云台升降中
} LifterPos;
typedef enum _PitchPos
{
  UP = 0,
  DOWN = 1,
} PitchPos;
typedef enum _NavMode
{
  PATROL = 0,
  TRACING = 1,
  RETREAT = 2,
  RESPONSE = 3,
  MANUAL = 4,
} NavMode;

typedef enum _NavGoal
{
  HOME = 0,
  BONUS = 1,
  OUTPOST = 2,
  OWN_FORT = 3,
  ENEMY_FORT = 4,
} NavGoal;

typedef enum _SentryStance
{
  ATTACK = 1,
  DEFEND = 2,
  MOVE = 3
} SentryStance;

typedef enum _ControlMode
{
  AUTO = 0,
  MANUAL_CONTROL = 1,
} ControlMode;

typedef enum _RobotID
{
  Hero = 1,
  Engineer = 2,
  Infantry1 = 3,
  Infantry2 = 4,
  Sentry = 5,
} RobotID;

struct AllyRobotInfo
{
  int robot_id;                       // 机器人ID
  int remain_hp;                      // 剩余血量
  geometry_msgs::msg::Pose position;  // 位置
};

struct EnemyRobotInfo
{
  int robot_id;                       // 机器人ID
  int remain_hp;                      // 剩余血量
  int allowed_projectile;             // 可打弹丸数
  geometry_msgs::msg::Pose position;  // 位置
};
struct AreaVizConfig
{
  std::string name;
  Area_Square area;
  std::array<float, 3> color;
};

struct PolygonVizConfig
{
  std::string name;
  std::vector<Point2D> vertices;
  std::array<float, 3> color;
};

struct CircleVizConfig
{
  std::string name;
  Area_Circle area;
  std::array<float, 3> color;
};

template <std::size_t N>
inline PolygonVizConfig makePolygonVizConfig(
  const std::string & name, const AreaPolygon<N, Point2D> & polygon, const std::array<float, 3> & color)
{
  return {name, {polygon.vertices.begin(), polygon.vertices.end()}, color};
}

}  // namespace Sentry_BT