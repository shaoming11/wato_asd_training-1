#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
 public:
  CostmapNode();

 private:
  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif
