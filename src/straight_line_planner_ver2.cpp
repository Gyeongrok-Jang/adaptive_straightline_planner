#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

// costmap 충돌 체크
bool StraightLine::checkCollision(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return true; // map 범위 벗어나면 충돌
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return (cost >= nav2_costmap_2d::LETHAL_OBSTACLE); // 장애물일 경우 true
}

// x 먼저 이동 시 충돌 체크
bool StraightLine::isXFirstSafe(const geometry_msgs::msg::PoseStamped & start,
                                const geometry_msgs::msg::PoseStamped & goal)
{
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;

  int x_steps = std::max(1, static_cast<int>(std::round(std::abs(dx) / interpolation_resolution_)));
  int y_steps = std::max(1, static_cast<int>(std::round(std::abs(dy) / interpolation_resolution_)));

  double x_inc = dx / x_steps;
  double y_inc = dy / y_steps;

  // x 이동
  for (int i = 1; i <= x_steps; ++i) {
    double x = start.pose.position.x + x_inc * i;
    double y = start.pose.position.y;
    if (checkCollision(x, y)) return false;
  }
  // y 이동
  for (int i = 1; i <= y_steps; ++i) {
    double x = goal.pose.position.x;
    double y = start.pose.position.y + y_inc * i;
    if (checkCollision(x, y)) return false;
  }
  return true;
}

// y 먼저 이동 시 충돌 체크
bool StraightLine::isYFirstSafe(const geometry_msgs::msg::PoseStamped & start,
                                const geometry_msgs::msg::PoseStamped & goal)
{
  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;

  int x_steps = std::max(1, static_cast<int>(std::round(std::abs(dx) / interpolation_resolution_)));
  int y_steps = std::max(1, static_cast<int>(std::round(std::abs(dy) / interpolation_resolution_)));

  double x_inc = dx / x_steps;
  double y_inc = dy / y_steps;

  // y 이동
  for (int i = 1; i <= y_steps; ++i) {
    double x = start.pose.position.x;
    double y = start.pose.position.y + y_inc * i;
    if (checkCollision(x, y)) return false;
  }
  // x 이동
  for (int i = 1; i <= x_steps; ++i) {
    double x = start.pose.position.x + x_inc * i;
    double y = goal.pose.position.y;
    if (checkCollision(x, y)) return false;
  }
  return true;
}

// 최종 경로 생성
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

  int x_steps = std::max(1, static_cast<int>(std::round(std::abs(dx) / interpolation_resolution_)));
  int y_steps = std::max(1, static_cast<int>(std::round(std::abs(dy) / interpolation_resolution_)));

  double x_inc = dx / x_steps;
  double y_inc = dy / y_steps;

  auto now = node_->now();  

  bool x_first_safe = isXFirstSafe(start, goal);
  bool y_first_safe = isYFirstSafe(start, goal);

  if (x_first_safe) {
    // x 먼저 → y 이동
    for (int i = 0; i < x_steps + y_steps; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.z = 0.0;
      double yaw = 0.0;

      if (i < x_steps) {
        pose.pose.position.x = start.pose.position.x + x_inc * i;
        pose.pose.position.y = start.pose.position.y;
        yaw = (dx >= 0) ? 0.0 : M_PI;
      } else {
        pose.pose.position.x = goal.pose.position.x;
        pose.pose.position.y = start.pose.position.y + y_inc * (i - x_steps);
        yaw = (dy >= 0) ? M_PI_2 : -M_PI_2;
      }

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      pose.header.stamp = now;
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  } else if (y_first_safe) {
    // y 먼저 → x 이동
    for (int i = 0; i < x_steps + y_steps; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.z = 0.0;
      double yaw = 0.0;

      if (i < y_steps) {
        pose.pose.position.x = start.pose.position.x;
        pose.pose.position.y = start.pose.position.y + y_inc * i;
        yaw = (dy >= 0) ? M_PI_2 : -M_PI_2;
      } else {
        pose.pose.position.x = start.pose.position.x + x_inc * (i - y_steps);
        pose.pose.position.y = goal.pose.position.y;
        yaw = (dx >= 0) ? 0.0 : M_PI;
      }

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      pose.header.stamp = now;
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Both x-first and y-first paths blocked, using default ZigZag");
    // 기본 Zig-Zag
    for (int i = 0; i < x_steps + y_steps; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.z = 0.0;
      double yaw = 0.0;
      if (i < x_steps) {
        pose.pose.position.x = start.pose.position.x + x_inc * i;
        pose.pose.position.y = start.pose.position.y;
        yaw = (dx >= 0) ? 0.0 : M_PI;
      } else {
        pose.pose.position.x = goal.pose.position.x;
        pose.pose.position.y = start.pose.position.y + y_inc * (i - x_steps);
        yaw = (dy >= 0) ? M_PI_2 : -M_PI_2;
      }
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      pose.header.stamp = now;
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  }

  // 마지막 목표 pose
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  tf2::Quaternion q_goal;
  q_goal.setRPY(0, 0, std::atan2(dy, dx));
  goal_pose.pose.orientation = tf2::toMsg(q_goal);
  goal_pose.header.stamp = now;
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  RCLCPP_WARN(node_->get_logger(), "StraightLine Plan DONE");

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
