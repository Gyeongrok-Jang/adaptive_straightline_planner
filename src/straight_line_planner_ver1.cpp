#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup() { }
void StraightLine::activate() { }
void StraightLine::deactivate() { }

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal not in %s frame", global_frame_.c_str());
    return global_path;
  }

  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  global_path.poses.clear();

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;

  // 중간 y 좌표 (ㄷ자 형태)
  double mid_y = start.pose.position.y + dy / 2.0;

  auto now = node_->now();  

  // 1. x 방향 이동
  int x_steps = std::max(1, static_cast<int>(std::round(std::abs(dx) / interpolation_resolution_)));
  double x_inc = dx / x_steps;
  for (int i = 0; i <= x_steps; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_inc * i;
    pose.pose.position.y = start.pose.position.y;
    pose.pose.position.z = 0.0;

    double yaw = (dx >= 0) ? 0.0 : M_PI;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    pose.header.stamp = now;
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  // 2. y 방향 이동
  int y_steps = std::max(1, static_cast<int>(std::round(std::abs(mid_y - start.pose.position.y) / interpolation_resolution_)));
  double y_inc = (mid_y - start.pose.position.y) / y_steps;
  for (int i = 1; i <= y_steps; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + dx; // x 고정
    pose.pose.position.y = start.pose.position.y + y_inc * i;
    pose.pose.position.z = 0.0;

    double yaw = (y_inc >= 0) ? M_PI_2 : -M_PI_2;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    pose.header.stamp = now;
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  // 3. 최종 x 방향 이동 (goal x)
  int final_x_steps = std::max(1, static_cast<int>(std::round(std::abs(goal.pose.position.x - (start.pose.position.x + dx)) / interpolation_resolution_)));
  double final_x_inc = (goal.pose.position.x - (start.pose.position.x + dx)) / final_x_steps;
  for (int i = 1; i <= final_x_steps; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + dx + final_x_inc * i;
    pose.pose.position.y = mid_y; // y 고정
    pose.pose.position.z = 0.0;

    double yaw = (final_x_inc >= 0) ? 0.0 : M_PI;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    pose.header.stamp = now;
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  // 마지막 goal 위치
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  tf2::Quaternion q_goal;
  q_goal.setRPY(0, 0, std::atan2(goal.pose.position.y - mid_y, goal.pose.position.x - (start.pose.position.x + dx)));
  goal_pose.pose.orientation = tf2::toMsg(q_goal);
  goal_pose.header.stamp = now;
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  RCLCPP_WARN(node_->get_logger(), "D-Shaped (ㄷ자) Plan DONE");

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
