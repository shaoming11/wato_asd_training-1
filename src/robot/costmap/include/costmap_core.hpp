#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cmath>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot {

class CostmapCore {
 public:
  explicit CostmapCore(const rclcpp::Logger& logger);

  void updateRobotPose(double x, double y, double theta);
  void initializeCostmap();
  void convertToGrid(double range, double angle, int& x_grid, int& y_grid) const;
  bool isValidCell(int x, int y) const;
  void markObstacle(int x, int y);
  void inflateObstacles();
  nav_msgs::msg::OccupancyGrid getOccupancyGrid(const rclcpp::Time& stamp) const;

 private:
  rclcpp::Logger logger_;

  double resolution_;       // meters per cell
  int width_;               // number of cells in x
  int height_;              // number of cells in y
  double origin_x_;         // world x of grid cell (0,0)
  double origin_y_;         // world y of grid cell (0,0)
  int obstacle_cost_;       // cost assigned to obstacle cells (0-100)
  double inflation_radius_; // inflation radius in meters

  double robot_x_, robot_y_, robot_theta_;

  std::vector<int8_t> data_;
  std::vector<std::pair<int, int>> obstacle_cells_;
};

}  // namespace robot

#endif
