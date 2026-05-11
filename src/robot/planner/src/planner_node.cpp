#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger())),
  state_(State::WAITING_FOR_GOAL),
  goal_tolerance_(0.5)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.setMap(*msg);
  if (state_ == State::NAVIGATING) {
    publishPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  planner_.setGoal(msg->point.x, msg->point.y);
  state_ = State::NAVIGATING;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_.setRobotPose(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void PlannerNode::timerCallback() {
  if (state_ != State::NAVIGATING) return;

  if (planner_.isGoalReached(goal_tolerance_)) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  publishPath();
}

void PlannerNode::publishPath() {
  if (!planner_.hasMap() || !planner_.hasGoal()) return;
  auto result = planner_.planPath(this->get_clock()->now());
  if (result) {
    path_pub_->publish(*result);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

