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

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.01)); // 1cm
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}

double StraightLine::calculateOptimalMidX(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  double y_threshold = 0.5;  // y 차이가 20cm 이하이면 mid_x를 현재 x로
  if (std::abs(goal.pose.position.y - start.pose.position.y) <= y_threshold) {
    return start.pose.position.x;
  }

  double buffer = 0.8;
  double x_min = std::min(start.pose.position.x, goal.pose.position.x) - buffer;
  double x_max = std::max(start.pose.position.x, goal.pose.position.x) + buffer;

  double sample_resolution = 0.01;  // 1cm 단위
  int n_min = 10;

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

  // 길이가 n_min 이상인 블록 중에서 start.x와 가장 가까운 값 선택
  double best_x = std::numeric_limits<double>::quiet_NaN();
  double min_dist = std::numeric_limits<double>::max();

  for (auto & block : zero_blocks) {
      if (block.size() >= n_min) {
          double candidate = block[block.size() / 2];

          // 1. mid_x 후보 y선이 안전한지 이미 확인됨
          // 2. 이제 goal.x로 이동하는 x 방향 경로 검증
          bool path_clear = true;
          int num_x_samples = static_cast<int>(std::abs(goal.pose.position.x - candidate) / sample_resolution) + 1;
          for (int i = 0; i <= num_x_samples; ++i) {
              double x = candidate + (goal.pose.position.x - candidate) * i / num_x_samples;
              double y = goal.pose.position.y; // x 이동 시 y는 goal.y 고정
              unsigned int mx, my;
              if (!costmap_->worldToMap(x, y, mx, my) || costmap_->getCost(mx, my) > 0) {
                  path_clear = false;
                  break;
              }
          }

          if (!path_clear) continue; // 충돌 예상 mid_x 후보 배제

          // start와 가장 가까운 후보 선택
          double dist = std::abs(candidate - start.pose.position.x);
          if (dist < min_dist) {
              min_dist = dist;
              best_x = candidate;
          }
      }
  }

  return best_x;  // 조건 만족 블록 중 start와 가장 가까운 중앙값 반환
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
