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

  auto now = node_->now();

  // 중간 Y 좌표 = 두 X 사이 중앙
  double mid_y = (start.pose.position.y + goal.pose.position.y) / 2.0;

  // --- 1️⃣ 시작점 (Start) ---
  geometry_msgs::msg::PoseStamped pose_start = start;
  pose_start.header.stamp = now;
  pose_start.header.frame_id = global_frame_;
  global_path.poses.push_back(pose_start);
  RCLCPP_INFO(node_->get_logger(), "Added START point");

  // --- 2️⃣ 점1 (X 중간 + 목표 Y) ---
  geometry_msgs::msg::PoseStamped pose1;
  pose1.pose.position.x = (start.pose.position.x + goal.pose.position.x) / 2.0; 
  pose1.pose.position.y = start.pose.position.y ;
  pose1.pose.position.z = 0.0;

  // 방향: 목표 X 위치에 따라 설정
  tf2::Quaternion q1;
  q1.setRPY(0, 0, (goal.pose.position.x >= start.pose.position.x) ? 0.0 : M_PI);
  pose1.pose.orientation = tf2::toMsg(q1);

  pose1.header.stamp = now;
  pose1.header.frame_id = global_frame_;
  global_path.poses.push_back(pose1);
  RCLCPP_INFO(node_->get_logger(), "Added POINT 1");

  // --- 3️⃣ 점2 (중간 Y, X는 목표 X) ---
  geometry_msgs::msg::PoseStamped pose2;
  pose2.pose.position.x = (start.pose.position.x + goal.pose.position.x) / 2.0; 
  pose2.pose.position.y = goal.pose.position.y;  // 목표 Y
  pose2.pose.position.z = 0.0;

  // 방향: 세로 이동 방향
  tf2::Quaternion q2;
  q2.setRPY(0, 0, M_PI_2);  // 항상 위쪽으로 향한다고 가정
  pose2.pose.orientation = tf2::toMsg(q2);

  pose2.header.stamp = now;
  pose2.header.frame_id = global_frame_;
  global_path.poses.push_back(pose2);
  RCLCPP_INFO(node_->get_logger(), "Added POINT 2");

  // --- 4️⃣ 목적지 (Goal) ---
  geometry_msgs::msg::PoseStamped pose_goal = goal;

  // 방향: 목표점을 향하도록 설정
  tf2::Quaternion q_goal;
  q_goal.setRPY(0, 0, std::atan2(goal.pose.position.y - pose2.pose.position.y, 
                                goal.pose.position.x - pose2.pose.position.x));
  pose_goal.pose.orientation = tf2::toMsg(q_goal);

  pose_goal.header.stamp = now;
  pose_goal.header.frame_id = global_frame_;
  global_path.poses.push_back(pose_goal);
  RCLCPP_INFO(node_->get_logger(), "Added GOAL point");

  RCLCPP_WARN(node_->get_logger(), "Single D-Shaped (ㄷ자) Plan DONE");

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
