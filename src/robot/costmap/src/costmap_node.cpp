#include "costmap_node.hpp"
#include "tf2/utils.h"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double theta = tf2::getYaw(msg->pose.pose.orientation);
  costmap_.updateRobotPose(x, y, theta);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.initializeCostmap();

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    double range = msg->ranges[i];
    if (range > msg->range_min && range < msg->range_max) {
      int x_grid, y_grid;
      costmap_.convertToGrid(range, angle, x_grid, y_grid);
      costmap_.markObstacle(x_grid, y_grid);
    }
  }

  costmap_.inflateObstacles();
  costmap_pub_->publish(costmap_.getOccupancyGrid(this->get_clock()->now()));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
