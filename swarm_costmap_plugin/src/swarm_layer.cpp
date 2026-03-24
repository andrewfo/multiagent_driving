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
    
  current_ = true;
}

void SwarmLayer::poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_poses_ = *msg;
}

void SwarmLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_poses_.poses.empty()) return;

  // Inform the costmap that the area around the other cars needs to be recalculated
  for (const auto & pose : last_poses_.poses) {
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
  if (last_poses_.poses.empty()) return;

  for (const auto & pose : last_poses_.poses) {
    unsigned int mx, my;
    // Convert the (x,y) world coordinates of the other car into map grid cells
    if (master_grid.worldToMap(pose.position.x, pose.position.y, mx, my)) {
      
      // Inject a lethal obstacle (254) into the costmap at the car's location
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      
      // Paint a 3x3 pixel box around it to account for car size
      for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            master_grid.setCost(mx + i, my + j, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

}  // namespace swarm_costmap_plugin