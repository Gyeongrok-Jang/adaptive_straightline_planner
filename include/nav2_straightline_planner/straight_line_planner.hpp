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
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;
  double interpolation_resolution_;
  double buffer_;        // mid_x 후보 영역 buffer
  double y_threshold_;   // y 차이 임계값
  int n_min_;            // 최소 연속 safe 블록 길이

  // 추가 기능
  bool checkCollision(double x, double y);
  bool isXFirstSafe(const geometry_msgs::msg::PoseStamped & start,
                    const geometry_msgs::msg::PoseStamped & goal);
  bool isYFirstSafe(const geometry_msgs::msg::PoseStamped & start,
                    const geometry_msgs::msg::PoseStamped & goal);
  double calculate_path_cost(const nav_msgs::msg::Path & path);

  // mid_x 계산
  double calculateOptimalMidX(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  double calculateOptimalMidXWithRobot(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double robot_x, double robot_y);

  double distanceThreshold_ = 0.3;
  bool stop_update_ = false;
  
  // --- A* 관련 파라미터 ---
  double heuristic_weight_;   // h(x) 가중치
  double cost_weight_;        // guide penalty 가중치
  double guide_weight_;
  
  bool unknown_is_lethal_;    // 미확인 영역 처리
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
