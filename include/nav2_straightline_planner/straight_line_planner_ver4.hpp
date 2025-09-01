#pragma once

#include <cmath>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"

namespace nav2_straightline_planner
{

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  bool checkCollision(double x, double y);
  

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  bool allow_diagonal_;
  double heuristic_weight_;
  double cost_weight_;
  bool unknown_is_lethal_;

  //-------------------------------------------------
  // Helper structures
  //-------------------------------------------------
  struct PQItem {
    std::pair<int,int> coord;
    double cost;
    bool operator<(const PQItem & other) const { return cost > other.cost; } // min-heap
  };

  //-------------------------------------------------
  // A* function for a subsegment
  //-------------------------------------------------
  nav_msgs::msg::Path aStarPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
};

} // namespace nav2_straightline_planner

PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
