#include "control_core.hpp"

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger),
  robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
  lookahead_distance_(1.0),
  linear_speed_(0.5),
  max_angular_speed_(1.0),
  goal_tolerance_(0.3)
{}

void ControlCore::setPath(const nav_msgs::msg::Path& path) {
  path_ = path;
}

void ControlCore::setRobotPose(double x, double y, double theta) {
  robot_x_ = x;
  robot_y_ = y;
  robot_theta_ = theta;
}

bool ControlCore::hasPath() const {
  return !path_.poses.empty();
}

bool ControlCore::isGoalReached() const {
  if (path_.poses.empty()) return false;
  const auto& goal = path_.poses.back().pose.position;
  return computeDistance(robot_x_, robot_y_, goal.x, goal.y) < goal_tolerance_;
}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const {
  double dx = x1 - x2;
  double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() const {
  if (path_.poses.empty()) return std::nullopt;

  // Find closest waypoint index to avoid backtracking
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    double d = computeDistance(robot_x_, robot_y_,
                               path_.poses[i].pose.position.x,
                               path_.poses[i].pose.position.y);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  // Starting from closest, find first waypoint at >= lookahead_distance
  for (size_t i = closest_idx; i < path_.poses.size(); ++i) {
    double d = computeDistance(robot_x_, robot_y_,
                               path_.poses[i].pose.position.x,
                               path_.poses[i].pose.position.y);
    if (d >= lookahead_distance_) {
      return path_.poses[i];
    }
  }

  // No point found at lookahead distance — use the last waypoint
  return path_.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity() {
  geometry_msgs::msg::Twist cmd;

  if (path_.poses.empty()) return cmd;

  if (isGoalReached()) {
    RCLCPP_INFO(logger_, "Goal reached, stopping.");
    return cmd;  // zero velocity
  }

  auto lookahead = findLookaheadPoint();
  if (!lookahead) return cmd;

  double lp_x = lookahead->pose.position.x;
  double lp_y = lookahead->pose.position.y;

  // Transform lookahead point into robot frame
  double dx = lp_x - robot_x_;
  double dy = lp_y - robot_y_;
  double local_x =  dx * std::cos(robot_theta_) + dy * std::sin(robot_theta_);
  double local_y = -dx * std::sin(robot_theta_) + dy * std::cos(robot_theta_);

  double ld = std::sqrt(local_x * local_x + local_y * local_y);
  if (ld < 1e-6) return cmd;

  // Pure pursuit: kappa = 2*sin(alpha)/ld, where sin(alpha) = local_y/ld
  double alpha = std::atan2(local_y, local_x);
  double kappa = 2.0 * std::sin(alpha) / ld;
  double angular_vel = linear_speed_ * kappa;

  // Clamp angular velocity
  angular_vel = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_vel));

  cmd.linear.x = linear_speed_;
  cmd.angular.z = angular_vel;
  return cmd;
}

}  // namespace robot

