#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger),
  resolution_(0.1),
  width_(2000),
  height_(2000),
  origin_x_(-100.0),
  origin_y_(-100.0),
  costmap_received_(false),
  robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
  last_update_x_(0.0), last_update_y_(0.0),
  distance_threshold_(1.5),
  should_update_(false)
{
  global_map_.header.frame_id = "map";
  global_map_.info.resolution = static_cast<float>(resolution_);
  global_map_.info.width = static_cast<uint32_t>(width_);
  global_map_.info.height = static_cast<uint32_t>(height_);
  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width_ * height_, -1);  // -1 = unknown
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap) {
  latest_costmap_ = costmap;
  costmap_received_ = true;
}

void MapMemoryCore::updateRobotPose(double x, double y, double theta) {
  robot_x_ = x;
  robot_y_ = y;
  robot_theta_ = theta;

  double dx = x - last_update_x_;
  double dy = y - last_update_y_;
  if (std::sqrt(dx * dx + dy * dy) >= distance_threshold_) {
    should_update_ = true;
  }
}

bool MapMemoryCore::shouldUpdateMap() const {
  return should_update_ && costmap_received_;
}

void MapMemoryCore::markUpdated() {
  last_update_x_ = robot_x_;
  last_update_y_ = robot_y_;
  should_update_ = false;
}

void MapMemoryCore::integrateCostmap() {
  const auto& cm = latest_costmap_;
  int cm_w = static_cast<int>(cm.info.width);
  int cm_h = static_cast<int>(cm.info.height);
  double cm_res = cm.info.resolution;
  double cm_ox = cm.info.origin.position.x;
  double cm_oy = cm.info.origin.position.y;

  double cos_t = std::cos(robot_theta_);
  double sin_t = std::sin(robot_theta_);

  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      int8_t cell_val = cm.data[cy * cm_w + cx];
      if (cell_val < 0) continue;  // unknown cell, skip

      // Center of this costmap cell in robot-local frame
      double local_x = (cx + 0.5) * cm_res + cm_ox;
      double local_y = (cy + 0.5) * cm_res + cm_oy;

      // Rotate and translate into global frame
      double global_x = robot_x_ + local_x * cos_t - local_y * sin_t;
      double global_y = robot_y_ + local_x * sin_t + local_y * cos_t;

      int gx = static_cast<int>((global_x - origin_x_) / resolution_);
      int gy = static_cast<int>((global_y - origin_y_) / resolution_);

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) continue;
      global_map_.data[gy * width_ + gx] = cell_val;
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

bool MapMemoryCore::hasCostmap() const {
  return costmap_received_;
}

}  // namespace robot

