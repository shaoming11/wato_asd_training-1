#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger),
  map_received_(false),
  goal_received_(false),
  robot_pose_received_(false),
  goal_x_(0.0), goal_y_(0.0),
  robot_x_(0.0), robot_y_(0.0),
  occupied_threshold_(90)
{}

void PlannerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
  map_ = map;
  map_received_ = true;
}

void PlannerCore::setGoal(double goal_x, double goal_y) {
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  goal_received_ = true;
}

void PlannerCore::setRobotPose(double x, double y) {
  robot_x_ = x;
  robot_y_ = y;
  robot_pose_received_ = true;
}

bool PlannerCore::hasMap() const { return map_received_; }
bool PlannerCore::hasGoal() const { return goal_received_; }

bool PlannerCore::isGoalReached(double tolerance) const {
  if (!robot_pose_received_ || !goal_received_) return false;
  double dx = robot_x_ - goal_x_;
  double dy = robot_y_ - goal_y_;
  return std::sqrt(dx * dx + dy * dy) < tolerance;
}

CellIndex PlannerCore::worldToGrid(double x, double y) const {
  double ox = map_.info.origin.position.x;
  double oy = map_.info.origin.position.y;
  double res = map_.info.resolution;
  return CellIndex(
    static_cast<int>((x - ox) / res),
    static_cast<int>((y - oy) / res));
}

geometry_msgs::msg::PoseStamped PlannerCore::gridToWorld(
  const CellIndex& idx, const rclcpp::Time& stamp) const
{
  double ox = map_.info.origin.position.x;
  double oy = map_.info.origin.position.y;
  double res = map_.info.resolution;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  pose.pose.position.x = (idx.x + 0.5) * res + ox;
  pose.pose.position.y = (idx.y + 0.5) * res + oy;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool PlannerCore::isValid(const CellIndex& idx) const {
  return idx.x >= 0 &&
         idx.x < static_cast<int>(map_.info.width) &&
         idx.y >= 0 &&
         idx.y < static_cast<int>(map_.info.height);
}

bool PlannerCore::isPassable(const CellIndex& idx) const {
  if (!isValid(idx)) return false;
  int8_t val = map_.data[static_cast<size_t>(idx.y) * map_.info.width + static_cast<size_t>(idx.x)];
  // val == -1 means unknown; treat as passable for exploration
  return val < static_cast<int8_t>(occupied_threshold_);
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

std::optional<nav_msgs::msg::Path> PlannerCore::planPath(const rclcpp::Time& stamp) {
  if (!map_received_ || !goal_received_) return std::nullopt;

  CellIndex start = worldToGrid(robot_x_, robot_y_);
  CellIndex goal = worldToGrid(goal_x_, goal_y_);

  if (!isValid(start) || !isValid(goal)) {
    RCLCPP_WARN(logger_, "Start or goal is outside map bounds");
    return std::nullopt;
  }

  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  // 8-connectivity: cardinal (cost 1) and diagonal (cost sqrt(2))
  static const int dx[] = {1, -1,  0,  0,  1,  1, -1, -1};
  static const int dy[] = {0,  0,  1, -1,  1, -1,  1, -1};
  static const double move_cost[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal) {
      return reconstructPath(came_from, goal, stamp);
    }

    for (int i = 0; i < 8; ++i) {
      CellIndex nb(current.index.x + dx[i], current.index.y + dy[i]);
      if (!isPassable(nb)) continue;

      double tentative_g = g_score[current.index] + move_cost[i];
      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nb] = tentative_g;
        came_from[nb] = current.index;
        open_set.emplace(nb, tentative_g + heuristic(nb, goal));
      }
    }
  }

  RCLCPP_WARN(logger_, "A* failed to find a path to goal (%.2f, %.2f)", goal_x_, goal_y_);
  return std::nullopt;
}

nav_msgs::msg::Path PlannerCore::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
  CellIndex goal,
  const rclcpp::Time& stamp) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = "map";

  std::vector<CellIndex> cells;
  CellIndex current = goal;
  while (came_from.count(current)) {
    cells.push_back(current);
    current = came_from.at(current);
  }
  cells.push_back(current);  // start cell
  std::reverse(cells.begin(), cells.end());

  for (const auto& cell : cells) {
    path.poses.push_back(gridToWorld(cell, stamp));
  }
  return path;
}

}  // namespace robot

