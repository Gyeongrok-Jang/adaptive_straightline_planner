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
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05)); // default 5cm
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}

/// -------------------------
/// 기존 X축 기준 mid_x 계산
/// -------------------------
double StraightLine::calculateOptimalMidX(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  double y_threshold = 0.5;
  if (std::abs(goal.pose.position.y - start.pose.position.y) <= y_threshold) {
    return start.pose.position.x;
  }

  double buffer = 0.8;
  double x_min = std::min(start.pose.position.x, goal.pose.position.x) - buffer;
  double x_max = std::max(start.pose.position.x, goal.pose.position.x) + buffer;

  double sample_resolution = 0.01;
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

  double best_x = std::numeric_limits<double>::quiet_NaN();
  double min_dist = std::numeric_limits<double>::max();

  for (auto & block : zero_blocks) {
    if (block.size() >= n_min) {
      double candidate = block[block.size() / 2];
      bool path_clear = true;
      int num_x_samples = static_cast<int>(std::abs(goal.pose.position.x - candidate) / sample_resolution) + 1;
      for (int i = 0; i <= num_x_samples; ++i) {
        double x = candidate + (goal.pose.position.x - candidate) * i / num_x_samples;
        double y = goal.pose.position.y;
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my) || costmap_->getCost(mx, my) > 0) {
          path_clear = false;
          break;
        }
      }
      if (!path_clear) continue;

      double dist = std::abs(candidate - start.pose.position.x);
      if (dist < min_dist) {
        min_dist = dist;
        best_x = candidate;
      }
    }
  }
  return best_x;
}

/// -------------------------
/// 새로 추가: Y축 기준 mid_y 계산
/// -------------------------
double StraightLine::calculateOptimalMidY(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  double x_threshold = 0.5;
  if (std::abs(goal.pose.position.x - start.pose.position.x) <= x_threshold) {
    return start.pose.position.y;
  }

  double buffer = 0.8;
  double y_min = std::min(start.pose.position.y, goal.pose.position.y) - buffer;
  double y_max = std::max(start.pose.position.y, goal.pose.position.y) + buffer;

  double sample_resolution = 0.01;
  int n_min = 10;

  std::vector<double> candidate_ys;
  for (double y = y_min; y <= y_max; y += sample_resolution)
    candidate_ys.push_back(y);

  std::vector<std::vector<double>> zero_blocks;
  std::vector<double> current_block;
  int num_x_samples = static_cast<int>(std::abs(goal.pose.position.x - start.pose.position.x) / sample_resolution) + 1;

  for (auto & y : candidate_ys) {
    bool collision = false;
    for (int j = 0; j <= num_x_samples; ++j) {
      double ratio = j / double(num_x_samples);
      double x = start.pose.position.x + ratio * (goal.pose.position.x - start.pose.position.x);
      unsigned int mx, my;
      if (!costmap_->worldToMap(x, y, mx, my) || costmap_->getCost(mx, my) > 0) {
        collision = true;
        break;
      }
    }
    if (!collision)
      current_block.push_back(y);
    else if (!current_block.empty()) {
      zero_blocks.push_back(current_block);
      current_block.clear();
    }
  }
  if (!current_block.empty())
    zero_blocks.push_back(current_block);

  double best_y = std::numeric_limits<double>::quiet_NaN();
  double min_dist = std::numeric_limits<double>::max();

  for (auto & block : zero_blocks) {
    if (block.size() >= n_min) {
      double candidate = block[block.size() / 2];
      bool path_clear = true;
      int num_y_samples = static_cast<int>(std::abs(goal.pose.position.y - candidate) / sample_resolution) + 1;
      for (int i = 0; i <= num_y_samples; ++i) {
        double y = candidate + (goal.pose.position.y - candidate) * i / num_y_samples;
        double x = goal.pose.position.x;
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my) || costmap_->getCost(mx, my) > 0) {
          path_clear = false;
          break;
        }
      }
      if (!path_clear) continue;

      double dist = std::abs(candidate - start.pose.position.y);
      if (dist < min_dist) {
        min_dist = dist;
        best_y = candidate;
      }
    }
  }
  return best_y;
}

/// ---------------------------------------------
/// 최종 경로 생성 (X 먼저 / Y 먼저를 각도 기반으로 선택)
/// ---------------------------------------------
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

  // base_link -> map TF 가져오기
  geometry_msgs::msg::TransformStamped tf_base;
  try {
      tf_base = tf_->lookupTransform(global_frame_, "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
      return path_msg;
  }

  // base_link yaw 계산
  tf2::Quaternion q(
      tf_base.transform.rotation.x,
      tf_base.transform.rotation.y,
      tf_base.transform.rotation.z,
      tf_base.transform.rotation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // yaw를 0~360도로 변환
  double yaw_deg = yaw * 180.0 / M_PI;
  if (yaw_deg < 0) yaw_deg += 360.0;

  // x_first 결정: yaw 구간 기준
  bool x_first = false;
  if ((yaw_deg >= 0 && yaw_deg <= 45) ||
      (yaw_deg >= 135 && yaw_deg <= 225) ||
      (yaw_deg >= 315 && yaw_deg <= 360)) {
      x_first = true;
  } else {
      x_first = false; // 그 외는 Y-first
  }


  double resolution = interpolation_resolution_;
  std::vector<geometry_msgs::msg::PoseStamped> temp_path;

  if (x_first) {
    // ===== X 먼저 =====
    double mid_x = calculateOptimalMidX(start, goal);
    if (std::isnan(mid_x)) {
      RCLCPP_WARN(node_->get_logger(), "No valid mid_x found.");
      return path_msg;
    }

    // 1) start -> (mid_x, start.y)
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

    // 2) (mid_x, start.y) -> (mid_x, goal.y)
    n_points = std::max(2, int(std::abs(goal.pose.position.y - start.pose.position.y) / resolution));
    for (int i = 1; i <= n_points; ++i) {
      double y = start.pose.position.y + (goal.pose.position.y - start.pose.position.y) * i / n_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = mid_x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q; q.setRPY(0,0,(goal.pose.position.y>=start.pose.position.y)?M_PI_2:-M_PI_2);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }

    // 3) (mid_x, goal.y) -> goal
    n_points = std::max(2, int(std::abs(goal.pose.position.x - mid_x) / resolution));
    for (int i = 1; i <= n_points; ++i) {
      double x = mid_x + (goal.pose.position.x - mid_x) * i / n_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.position.y = goal.pose.position.y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q; q.setRPY(0,0,0.0);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }

  } else {
    // ===== Y 먼저 =====
    double mid_y = calculateOptimalMidY(start, goal);
    if (std::isnan(mid_y)) {
      RCLCPP_WARN(node_->get_logger(), "No valid mid_y found.");
      return path_msg;
    }

    // 1) start -> (start.x, mid_y)
    int n_points = std::max(2, int(std::abs(mid_y - start.pose.position.y) / resolution));
    for (int i = 0; i <= n_points; ++i) {
      double y = start.pose.position.y + (mid_y - start.pose.position.y) * i / n_points;
      geometry_msgs::msg::PoseStamped pose = start;
      pose.pose.position.x = start.pose.position.x;
      pose.pose.position.y = y;
      tf2::Quaternion q; q.setRPY(0,0,(mid_y>=start.pose.position.y)?M_PI_2:-M_PI_2);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }

    // 2) (start.x, mid_y) -> (goal.x, mid_y)
    n_points = std::max(2, int(std::abs(goal.pose.position.x - start.pose.position.x) / resolution));
    for (int i = 1; i <= n_points; ++i) {
      double x = start.pose.position.x + (goal.pose.position.x - start.pose.position.x) * i / n_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.position.y = mid_y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q; q.setRPY(0,0,(goal.pose.position.x>=start.pose.position.x)?0.0:M_PI);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }

    // 3) (goal.x, mid_y) -> goal
    n_points = std::max(2, int(std::abs(goal.pose.position.y - mid_y) / resolution));
    for (int i = 1; i <= n_points; ++i) {
      double y = mid_y + (goal.pose.position.y - mid_y) * i / n_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = goal.pose.position.x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q; q.setRPY(0,0,0.0);
      pose.pose.orientation = tf2::toMsg(q);
      temp_path.push_back(pose);
    }
  }

  path_msg.poses = temp_path;
  RCLCPP_INFO(node_->get_logger(), 
    "Final path generated with %s-first strategy: %zu poses",
    x_first ? "X" : "Y", path_msg.poses.size());

  return path_msg;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
