/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *********************************************************************/

#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp" // planner mode
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_straightline_planner
{

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  // Planner interface
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

private:
  // --- Core members ---
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;
  std::string name_;

  double interpolation_resolution_ = 0.01;  // 기본 1cm
  double buffer_ = 0.8;                      // mid 후보 영역 buffer
  double y_threshold_ = 0.5;                 // y 차이 임계값
  int n_min_ = 10;                           // 최소 연속 safe 블록 길이
  double distanceThreshold_ = 0.3;
  bool stop_update_ = false;

  // --- Collision & Safety ---
  bool checkCollision(double x, double y);
  bool isXFirstSafe(const geometry_msgs::msg::PoseStamped & start,
                    const geometry_msgs::msg::PoseStamped & goal);
  bool isYFirstSafe(const geometry_msgs::msg::PoseStamped & start,
                    const geometry_msgs::msg::PoseStamped & goal);

  // --- Path / Mid calculations ---
  double calculateOptimalMidX(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  double calculateOptimalMidY(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  double calculateOptimalMidXWithRobot(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double robot_x, double robot_y);

  double calculateOptimalMidYWithRobot(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double robot_x, double robot_y);

  // --- Path cost 계산 (optional) ---
  double calculate_path_cost(const nav_msgs::msg::Path & path);

  // --- Subscriber ---
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
  double latest_yaw_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
  bool use_astar_ = false;  // false -> StraightLine, true -> A*

  // --- A* 플래너 포인터 ---
  std::shared_ptr<nav2_core::GlobalPlanner> astar_planner_;
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
