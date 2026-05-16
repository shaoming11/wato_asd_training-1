#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot {

// 2D grid index
struct CellIndex {
  int x, y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex& o) const { return x == o.x && y == o.y; }
  bool operator!=(const CellIndex& o) const { return !(*this == o); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
 public:
  explicit PlannerCore(const rclcpp::Logger& logger);

  void setMap(const nav_msgs::msg::OccupancyGrid& map);
  void setGoal(double goal_x, double goal_y);
  void setRobotPose(double x, double y);
  bool hasMap() const;
  bool hasGoal() const;
  bool isGoalReached(double tolerance) const;

  std::optional<nav_msgs::msg::Path> planPath(const rclcpp::Time& stamp);

 private:
  rclcpp::Logger logger_;

  nav_msgs::msg::OccupancyGrid map_;
  bool map_received_;
  bool goal_received_;
  bool robot_pose_received_;

  double goal_x_, goal_y_;
  double robot_x_, robot_y_;
  int occupied_threshold_;

  CellIndex worldToGrid(double x, double y) const;
  geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex& idx, const rclcpp::Time& stamp) const;
  bool isValid(const CellIndex& idx) const;
  bool isPassable(const CellIndex& idx) const;
  double heuristic(const CellIndex& a, const CellIndex& b) const;
  nav_msgs::msg::Path reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    CellIndex goal,
    const rclcpp::Time& stamp) const;
};

}  // namespace robot

#endif

