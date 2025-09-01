#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // --- Declare parameters with defaults matching previous hardcoded values ---
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".buffer", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".y_threshold", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".n_min", rclcpp::ParameterValue(10));

  // --- Get parameters from YAML ---
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".buffer", buffer_);
  node_->get_parameter(name_ + ".y_threshold", y_threshold_);
  node_->get_parameter(name_ + ".n_min", n_min_);
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}

double StraightLine::calculateOptimalMidX(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (std::abs(goal.pose.position.y - start.pose.position.y) <= y_threshold_) {
    return start.pose.position.x;
  }

  double x_min = std::min(start.pose.position.x, goal.pose.position.x) - buffer_;
  double x_max = std::max(start.pose.position.x, goal.pose.position.x) + buffer_;

  double sample_resolution = 0.01;
  
  std::vector<double> candidate_xs;
  for (double x = x_min; x <= x_max; x += sample_resolution)
    candidate_xs.push_back(x);

  std::vector<std::vector<double>> zero_blocks;
  std::vector<double> current_block;
  int num_y_samples = static_cast<int>(std::abs(goal.pose.position.y - start.pose.position.y) / sample_resolution) + 1;

  for (auto & x : candidate_xs) {
    bool collision = false;
    for (int j = 0; j <= num_y_samples; ++j) {
      double ratio = j / double(num_y_samples);
      double y = start.pose.position.y + ratio * (goal.pose.position.y - start.pose.position.y);
      unsigned int mx, my;
      if (!costmap_->worldToMap(x, y, mx, my) || costmap_->getCost(mx, my) > 0) {
        collision = true;
        break;
      }
    }
    if (!collision)
      current_block.push_back(x);
    else if (!current_block.empty()) {
      zero_blocks.push_back(current_block);
      current_block.clear();
    }
  }
  if (!current_block.empty())
    zero_blocks.push_back(current_block);

  double best_x = std::numeric_limits<double>::quiet_NaN();
  double min_dist = std::numeric_limits<double>::max();

  for (auto & block : zero_blocks) {
    if (block.size() >= n_min_) {
      double candidate = block[block.size() / 2];
      double dist = std::abs(candidate - start.pose.position.x);
      if (dist < min_dist) {
        min_dist = dist;
        best_x = candidate;
      }
    }
  }

  return best_x;
}

// ---------------------------------------------
// 최종 경로 생성 (ㄷ자)
// ---------------------------------------------
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = global_frame_;
  path_msg.header.stamp = node_->now();
  path_msg.poses.clear();

  if (!costmap_) {
    RCLCPP_ERROR(node_->get_logger(), "Costmap not set");
    return path_msg;
  }

  // mid_x 계산
  double mid_x = calculateOptimalMidX(start, goal);
  if (std::isnan(mid_x)) {
    RCLCPP_WARN(node_->get_logger(), "No valid mid_x found. D-shaped path not created.");
    return path_msg;
  }

  // ㄷ자 경로를 촘촘하게 포인트 생성
  double resolution = interpolation_resolution_;  // 예: 0.05 ~ 0.1 m 간격
  std::vector<geometry_msgs::msg::PoseStamped> temp_path;

  // 1) start -> (mid_x, start.y)
  {
    int n_points = std::max(2, int(std::abs(mid_x - start.pose.position.x) / resolution));
    for (int i = 0; i <= n_points; ++i) {
      double x = start.pose.position.x + (mid_x - start.pose.position.x) * i / n_points;
      geometry_msgs::msg::PoseStamped pose = start;
      pose.pose.position.x = x;
      pose.pose.position.y = start.pose.position.y;
      tf2::Quaternion q; q.setRPY(0,0,(mid_x>=start.pose.position.x)?0.0:M_PI);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }
  }

  // 2) (mid_x, start.y) -> (mid_x, goal.y)
  {
    int n_points = std::max(2, int(std::abs(goal.pose.position.y - start.pose.position.y) / resolution));
    for (int i = 1; i <= n_points; ++i) { // i=1로 시작해서 중복 방지
      double y = start.pose.position.y + (goal.pose.position.y - start.pose.position.y) * i / n_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = mid_x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q; q.setRPY(0,0,(goal.pose.position.y>=start.pose.position.y)?M_PI_2:-M_PI_2);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }
  }

  // 3) (mid_x, goal.y) -> goal
  {
    int n_points = std::max(2, int(std::abs(goal.pose.position.x - mid_x) / resolution));
    for (int i = 1; i <= n_points; ++i) {
      double x = mid_x + (goal.pose.position.x - mid_x) * i / n_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.position.y = goal.pose.position.y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0,0,std::atan2(goal.pose.position.y - goal.pose.position.y, goal.pose.position.x - mid_x));
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }
  }

  path_msg.poses = temp_path;
  RCLCPP_WARN(node_->get_logger(), "Final D-shaped dense path DONE: %zu poses", path_msg.poses.size());
  return path_msg;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
