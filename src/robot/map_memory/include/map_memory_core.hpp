#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cmath>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot {

class MapMemoryCore {
 public:
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  void updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
  void updateRobotPose(double x, double y, double theta);
  bool shouldUpdateMap() const;
  void integrateCostmap();
  void markUpdated();
  nav_msgs::msg::OccupancyGrid getGlobalMap() const;
  bool hasCostmap() const;

 private:
  rclcpp::Logger logger_;

  double resolution_;
  int width_;
  int height_;
  double origin_x_;
  double origin_y_;

  nav_msgs::msg::OccupancyGrid global_map_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool costmap_received_;

  double robot_x_, robot_y_, robot_theta_;
  double last_update_x_, last_update_y_;
  double distance_threshold_;
  bool should_update_;
};

}  // namespace robot

#endif

