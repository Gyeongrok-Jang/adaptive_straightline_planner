#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <limits>

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

// ---------------------------------------------
// 후보 mid_x 중 최적 경로를 선택하는 함수 (±1.5m 여유 포함)
// ---------------------------------------------
double StraightLine::calculateOptimalMidX(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  int num_divisions = 10; // 후보 개수
  double best_mid_x = start.pose.position.x;
  double min_cost = std::numeric_limits<double>::max();

  // 탐색 범위: start와 goal X ±1.5 m
  double buffer = 1.5;
  double x_min = std::min(start.pose.position.x, goal.pose.position.x) - buffer;
  double x_max = std::max(start.pose.position.x, goal.pose.position.x) + buffer;

  for (int i = 0; i < num_divisions; ++i) {
    double candidate_x = x_min + i / double(num_divisions - 1) * (x_max - x_min);

    // Y 방향 샘플링
    int num_points = 20;
    double cost = 0.0;
    for (int j = 0; j <= num_points; ++j) {
      double ratio = j / double(num_points);
      double y = start.pose.position.y + ratio * (goal.pose.position.y - start.pose.position.y);

      unsigned int mx, my;
      if (costmap_->worldToMap(candidate_x, y, mx, my)) {
        cost += costmap_->getCost(mx, my);
      } else {
        cost += 255; // 맵 밖이면 최대 패널티
      }
    }

    // goal과의 거리
    double dist_to_goal = std::abs(candidate_x - goal.pose.position.x);

    // 최적 후보 선택 조건:
    // 1) cost가 낮으면 무조건 갱신
    // 2) cost가 거의 같으면 goal에 더 가까운 쪽 선택
    if (cost < min_cost - 1e-6) {  
      min_cost = cost;
      best_mid_x = candidate_x;
    } else if (std::abs(cost - min_cost) < 1e-6) {
      double mid_target_x = (start.pose.position.x + goal.pose.position.x) / 2.0;
      double current_best_dist = std::abs(best_mid_x - mid_target_x);
      double dist_to_mid = std::abs(candidate_x - mid_target_x);
      if (dist_to_mid < current_best_dist) {
        best_mid_x = candidate_x;
      }
    }
  }

  return best_mid_x;
}

// ---------------------------------------------
// 경로 생성
// ---------------------------------------------
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

  // ---------------------------------------------
  // 최적 mid_x 계산 (장애물 패널티 기반)
  // ---------------------------------------------
  double mid_x = calculateOptimalMidX(start, goal);

  // --- 1️⃣ 시작점 (Start) ---
  geometry_msgs::msg::PoseStamped pose_start = start;
  pose_start.header.stamp = now;
  pose_start.header.frame_id = global_frame_;
  global_path.poses.push_back(pose_start);
  RCLCPP_INFO(node_->get_logger(), "Added START point");

  // --- 2️⃣ 점1 (X 중간 + 시작 Y) ---
  geometry_msgs::msg::PoseStamped pose1;
  pose1.pose.position.x = mid_x;
  pose1.pose.position.y = start.pose.position.y;
  pose1.pose.position.z = 0.0;

  tf2::Quaternion q1;
  q1.setRPY(0, 0, (goal.pose.position.x >= start.pose.position.x) ? 0.0 : M_PI);
  pose1.pose.orientation = tf2::toMsg(q1);

  pose1.header.stamp = now;
  pose1.header.frame_id = global_frame_;
  global_path.poses.push_back(pose1);
  RCLCPP_INFO(node_->get_logger(), "Added POINT 1");

  // --- 3️⃣ 점2 (X 중간 + 목표 Y) ---
  geometry_msgs::msg::PoseStamped pose2;
  pose2.pose.position.x = mid_x;
  pose2.pose.position.y = goal.pose.position.y;
  pose2.pose.position.z = 0.0;

  tf2::Quaternion q2;
  q2.setRPY(0, 0, M_PI_2); // 세로 방향
  pose2.pose.orientation = tf2::toMsg(q2);

  pose2.header.stamp = now;
  pose2.header.frame_id = global_frame_;
  global_path.poses.push_back(pose2);
  RCLCPP_INFO(node_->get_logger(), "Added POINT 2");

  // ★ 수정 후: 현재 로봇 위치와 비교
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
        global_frame_,   // target_frame (map)
        "base_link",     // source_frame (로봇)
        tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not get robot pose: %s", ex.what());
    return global_path;
  }

  // 현재 로봇 위치
  double robot_x = transform.transform.translation.x;
  double robot_y = transform.transform.translation.y;

  // 점1과 점2까지 거리 계산
  double dist1 = std::hypot(pose1.pose.position.x - robot_x,
                            pose1.pose.position.y - robot_y);
  double dist2 = std::hypot(pose2.pose.position.x - robot_x,
                            pose2.pose.position.y - robot_y);

  // 30cm 이내이면 경로 생성 중단
  if(dist1 < 0.3 && dist2 < 0.3) {
      RCLCPP_WARN(node_->get_logger(), "POINT1 & POINT2 too close to robot, skipping update!");
      return global_path;
  }

  // --- 4️⃣ 목표점 (Goal) ---
  geometry_msgs::msg::PoseStamped pose_goal = goal;
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
