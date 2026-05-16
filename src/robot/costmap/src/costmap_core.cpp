#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
: logger_(logger),
  resolution_(0.1),
  width_(500),
  height_(500),
  origin_x_(-25.0),
  origin_y_(-25.0),
  obstacle_cost_(100),
  inflation_radius_(2.0),
  robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0)
{
  data_.resize(width_ * height_, 0);
}

void CostmapCore::updateRobotPose(double x, double y, double theta) {
  robot_x_ = x;
  robot_y_ = y;
  robot_theta_ = theta;
}

void CostmapCore::initializeCostmap() {
  std::fill(data_.begin(), data_.end(), 0);
  obstacle_cells_.clear();
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) const {
  double local_x = range * std::cos(angle);
  double local_y = range * std::sin(angle);
  double world_x = robot_x_ + local_x * std::cos(robot_theta_) - local_y * std::sin(robot_theta_);
  double world_y = robot_y_ + local_x * std::sin(robot_theta_) + local_y * std::cos(robot_theta_);
  x_grid = static_cast<int>((world_x - origin_x_) / resolution_);
  y_grid = static_cast<int>((world_y - origin_y_) / resolution_);
}

bool CostmapCore::isValidCell(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

void CostmapCore::markObstacle(int x, int y) {
  if (!isValidCell(x, y)) return;
  data_[y * width_ + x] = static_cast<int8_t>(obstacle_cost_);
  obstacle_cells_.emplace_back(x, y);
}

void CostmapCore::inflateObstacles() {
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  for (const auto& [ox, oy] : obstacle_cells_) {
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (!isValidCell(nx, ny)) continue;
        double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
        if (dist > inflation_radius_) continue;
        auto cost = static_cast<int8_t>(obstacle_cost_ * (1.0 - dist / inflation_radius_));
        int idx = ny * width_ + nx;
        if (data_[idx] < cost) {
          data_[idx] = cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid(const rclcpp::Time& stamp) const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = "sim_world";
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = static_cast<uint32_t>(width_);
  grid.info.height = static_cast<uint32_t>(height_);
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data = data_;
  return grid;
}

}  // namespace robot
