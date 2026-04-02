#include "swarm_costmap_plugin/swarm_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

// Register the plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(swarm_costmap_plugin::SwarmLayer, nav2_costmap_2d::Layer)

namespace swarm_costmap_plugin
{

SwarmLayer::SwarmLayer() {}

void SwarmLayer::onInitialize()
{
  auto node = node_.lock(); 
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }
  
  // Subscribe to the local topic published by the Python websocket client
  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/swarm_poses", 10,
    std::bind(&SwarmLayer::poseCallback, this, std::placeholders::_1));

  // Subscribe to shared obstacles reported by other cars
  obstacle_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "/swarm_obstacles", 10,
    std::bind(&SwarmLayer::obstacleCallback, this, std::placeholders::_1));

  current_ = true;
}

void SwarmLayer::poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_poses_ = *msg;
}

void SwarmLayer::obstacleCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_obstacles_ = *msg;
}

void SwarmLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_poses_.poses.empty() && last_obstacles_.poses.empty()) return;

  // Expand bounds around other cars
  for (const auto & pose : last_poses_.poses) {
    *min_x = std::min(*min_x, pose.position.x - 0.5);
    *min_y = std::min(*min_y, pose.position.y - 0.5);
    *max_x = std::max(*max_x, pose.position.x + 0.5);
    *max_y = std::max(*max_y, pose.position.y + 0.5);
  }

  // Expand bounds around shared obstacles
  for (const auto & pose : last_obstacles_.poses) {
    *min_x = std::min(*min_x, pose.position.x - 0.5);
    *min_y = std::min(*min_y, pose.position.y - 0.5);
    *max_x = std::max(*max_x, pose.position.x + 0.5);
    *max_y = std::max(*max_y, pose.position.y + 0.5);
  }
}

void SwarmLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_poses_.poses.empty() && last_obstacles_.poses.empty()) return;

  const int grid_w = static_cast<int>(master_grid.getSizeInCellsX());
  const int grid_h = static_cast<int>(master_grid.getSizeInCellsY());

  // Stamp oriented rectangular footprint matching the real car (0.3m × 0.09m)
  // rotated by each car's yaw extracted from the PoseArray quaternion.
  const double res     = master_grid.getResolution();
  const double orig_x  = master_grid.getOriginX();
  const double orig_y  = master_grid.getOriginY();

  // Half-extents matching nav2.yaml footprint: [[0.15,0.045],…]
  constexpr double half_len = 0.15;   // 0.3 m / 2
  constexpr double half_wid = 0.045;  // 0.09 m / 2

  for (const auto & pose : last_poses_.poses) {
    const double cx  = pose.position.x;
    const double cy  = pose.position.y;
    // Extract yaw from quaternion (2D: only z and w are non-zero)
    const double qz  = pose.orientation.z;
    const double qw  = pose.orientation.w;
    const double yaw = 2.0 * std::atan2(qz, qw);
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);

    // AABB of the rotated box uses its diagonal as a conservative radius
    const double diag = std::hypot(half_len, half_wid);
    const int gx_min = std::max(min_i,
      static_cast<int>(std::floor((cx - diag - orig_x) / res)));
    const int gx_max = std::min(max_i - 1,
      static_cast<int>(std::ceil( (cx + diag - orig_x) / res)));
    const int gy_min = std::max(min_j,
      static_cast<int>(std::floor((cy - diag - orig_y) / res)));
    const int gy_max = std::min(max_j - 1,
      static_cast<int>(std::ceil( (cy + diag - orig_y) / res)));

    for (int gx = gx_min; gx <= gx_max; ++gx) {
      for (int gy = gy_min; gy <= gy_max; ++gy) {
        if (gx < 0 || gx >= grid_w || gy < 0 || gy >= grid_h) continue;
        // Cell centre in world coords
        const double wx = orig_x + (gx + 0.5) * res;
        const double wy = orig_y + (gy + 0.5) * res;
        // Rotate into car-local frame and test OBB membership
        const double dx = wx - cx;
        const double dy = wy - cy;
        const double lx =  dx * cos_y + dy * sin_y;
        const double ly = -dx * sin_y + dy * cos_y;
        if (std::abs(lx) <= half_len && std::abs(ly) <= half_wid) {
          master_grid.setCost(
            static_cast<unsigned int>(gx),
            static_cast<unsigned int>(gy),
            nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  // Mark shared obstacles as lethal (single cell per obstacle point)
  for (const auto & pose : last_obstacles_.poses) {
    unsigned int mx, my;
    if (master_grid.worldToMap(pose.position.x, pose.position.y, mx, my)) {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

}  // namespace swarm_costmap_plugin