#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot {

class ControlCore {
 public:
  explicit ControlCore(const rclcpp::Logger& logger);

  void setPath(const nav_msgs::msg::Path& path);
  void setRobotPose(double x, double y, double theta);
  bool hasPath() const;
  bool isGoalReached() const;
  geometry_msgs::msg::Twist computeVelocity();

 private:
  rclcpp::Logger logger_;

  nav_msgs::msg::Path path_;
  double robot_x_, robot_y_, robot_theta_;
  double lookahead_distance_;
  double linear_speed_;
  double max_angular_speed_;
  double goal_tolerance_;

  double computeDistance(double x1, double y1, double x2, double y2) const;
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
};

}  // namespace robot

#endif

