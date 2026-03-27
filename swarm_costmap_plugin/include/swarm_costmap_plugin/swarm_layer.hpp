#ifndef SWARM_LAYER_HPP_
#define SWARM_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <mutex>

namespace swarm_costmap_plugin
{

class SwarmLayer : public nav2_costmap_2d::Layer
{
public:
  SwarmLayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void reset() {}
  virtual bool isClearable() {return true;}
  virtual void matchSize() {}

private:
  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void obstacleCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  geometry_msgs::msg::PoseArray last_poses_;
  geometry_msgs::msg::PoseArray last_obstacles_;
  std::mutex mutex_;
};

}  // namespace swarm_costmap_plugin

#endif  // SWARM_LAYER_HPP_