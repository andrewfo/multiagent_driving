#include "swarm_costmap_plugin/swarm_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

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

  // Mark other cars as lethal obstacles (3x3 box for car size)
  for (const auto & pose : last_poses_.poses) {
    unsigned int mx, my;
    if (master_grid.worldToMap(pose.position.x, pose.position.y, mx, my)) {
      for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
          int cx = static_cast<int>(mx) + i;
          int cy = static_cast<int>(my) + j;
          if (cx >= 0 && cx < grid_w && cy >= 0 && cy < grid_h) {
            master_grid.setCost(
              static_cast<unsigned int>(cx),
              static_cast<unsigned int>(cy),
              nav2_costmap_2d::LETHAL_OBSTACLE);
          }
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