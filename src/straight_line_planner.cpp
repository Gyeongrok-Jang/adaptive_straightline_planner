#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "pluginlib/class_loader.hpp"

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
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  yaw_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/closest_obstacle_yaw", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      latest_yaw_ = msg->data;
    });

  mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/planner_mode", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      use_astar_ = msg->data;
      RCLCPP_INFO(node_->get_logger(), "Planner mode switched: %s", use_astar_ ? "A*" : "StraightLine");
    });

  // --- A* 플래너 생성 ---
  try {
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
    astar_planner_ = loader.createSharedInstance("nav2_navfn_planner/NavfnPlanner");
    astar_planner_->configure(parent, "astar", tf, costmap_ros);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create A* planner: %s", ex.what());
    astar_planner_.reset();
  }
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}


/// -------------------------
/// 기존 X축 기준 mid_x 계산
/// -------------------------
/// -------------------------
/// 회전 좌표계에서 mid_u 계산
/// -------------------------
double StraightLine::calculateOptimalMidX(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // yaw 방향 좌표계 정의
  Eigen::Vector2d dir(cos(latest_yaw_), sin(latest_yaw_));
  Eigen::Vector2d ortho(-sin(latest_yaw_), cos(latest_yaw_));
  auto toLocal = [&](double x, double y){
    Eigen::Vector2d p(x,y);
    return Eigen::Vector2d(p.dot(dir), p.dot(ortho));
  };
  auto toWorld = [&](double u, double v){
    Eigen::Vector2d p = u*dir + v*ortho;
    return std::pair<double,double>(p.x(), p.y());
  };

  Eigen::Vector2d s = toLocal(start.pose.position.x, start.pose.position.y);
  Eigen::Vector2d g = toLocal(goal.pose.position.x, goal.pose.position.y);

  // 기존 로직을 x 대신 u, y 대신 v 사용
  double v_threshold = 0.5;
  if (std::abs(g.y() - s.y()) <= v_threshold) {
    return s.x();
  }

  double buffer = 0.8;
  double u_min = std::min(s.x(), g.x()) - buffer;
  double u_max = std::max(s.x(), g.x()) + buffer;

  double sample_resolution = 0.01;
  int n_min = 10;

  std::vector<double> candidate_us;
  for (double u = u_min; u <= u_max; u += sample_resolution)
    candidate_us.push_back(u);

  std::vector<std::vector<double>> zero_blocks;
  std::vector<double> current_block;
  int num_v_samples = static_cast<int>(std::abs(g.y() - s.y()) / sample_resolution) + 1;

  for (auto & u : candidate_us) {
    bool collision = false;
    for (int j = 0; j <= num_v_samples; ++j) {
      double ratio = j / double(num_v_samples);
      double v = s.y() + ratio * (g.y() - s.y());
      auto [wx, wy] = toWorld(u,v);
      unsigned int mx,my;
      if (!costmap_->worldToMap(wx, wy, mx, my) || costmap_->getCost(mx,my) > 0) {
        collision = true;
        break;
      }
    }
    if (!collision) current_block.push_back(u);
    else if (!current_block.empty()) {
      zero_blocks.push_back(current_block);
      current_block.clear();
    }
  }
  if (!current_block.empty()) zero_blocks.push_back(current_block);

  double best_u = std::numeric_limits<double>::quiet_NaN();
  double min_dist = std::numeric_limits<double>::max();

  for (auto & block : zero_blocks) {
    if (block.size() >= n_min) {
      double candidate = block[block.size()/2];
      bool path_clear = true;
      int num_u_samples = static_cast<int>(std::abs(g.x() - candidate) / sample_resolution) + 1;
      for (int i=0; i<=num_u_samples; ++i) {
        double u = candidate + (g.x() - candidate) * i / num_u_samples;
        double v = g.y();
        auto [wx, wy] = toWorld(u,v);
        unsigned int mx,my;
        if (!costmap_->worldToMap(wx,wy,mx,my) || costmap_->getCost(mx,my)>0) {
          path_clear = false; break;
        }
      }
      if (!path_clear) continue;
      double dist = std::abs(candidate - s.x());
      if (dist < min_dist) {
        min_dist = dist; best_u = candidate;
      }
    }
  }
  return best_u;
}

/// -------------------------
/// 회전 좌표계에서 mid_v 계산
/// -------------------------
double StraightLine::calculateOptimalMidY(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  Eigen::Vector2d dir(cos(latest_yaw_), sin(latest_yaw_));
  Eigen::Vector2d ortho(-sin(latest_yaw_), cos(latest_yaw_));
  auto toLocal = [&](double x, double y){
    Eigen::Vector2d p(x,y);
    return Eigen::Vector2d(p.dot(dir), p.dot(ortho));
  };
  auto toWorld = [&](double u, double v){
    Eigen::Vector2d p = u*dir + v*ortho;
    return std::pair<double,double>(p.x(), p.y());
  };

  Eigen::Vector2d s = toLocal(start.pose.position.x, start.pose.position.y);
  Eigen::Vector2d g = toLocal(goal.pose.position.x, goal.pose.position.y);

  double u_threshold = 0.5;
  if (std::abs(g.x() - s.x()) <= u_threshold) {
    return s.y();
  }

  double buffer = 0.5;
  double v_min = std::min(s.y(), g.y()) - buffer;
  double v_max = std::max(s.y(), g.y()) + buffer;

  double sample_resolution = 0.01;
  int n_min = 10;

  std::vector<double> candidate_vs;
  for (double v = v_min; v <= v_max; v += sample_resolution)
    candidate_vs.push_back(v);

  std::vector<std::vector<double>> zero_blocks;
  std::vector<double> current_block;
  int num_u_samples = static_cast<int>(std::abs(g.x() - s.x()) / sample_resolution) + 1;

  for (auto & v : candidate_vs) {
    bool collision = false;
    for (int j = 0; j <= num_u_samples; ++j) {
      double ratio = j / double(num_u_samples);
      double u = s.x() + ratio * (g.x() - s.x());
      auto [wx, wy] = toWorld(u,v);
      unsigned int mx,my;
      if (!costmap_->worldToMap(wx, wy, mx, my) || costmap_->getCost(mx,my) > 0) {
        collision = true;
        break;
      }
    }
    if (!collision) current_block.push_back(v);
    else if (!current_block.empty()) {
      zero_blocks.push_back(current_block);
      current_block.clear();
    }
  }
  if (!current_block.empty()) zero_blocks.push_back(current_block);

  double best_v = std::numeric_limits<double>::quiet_NaN();
  double min_dist = std::numeric_limits<double>::max();

  for (auto & block : zero_blocks) {
    if (block.size() >= n_min) {
      double candidate = block[block.size()/2];
      bool path_clear = true;
      int num_v_samples = static_cast<int>(std::abs(g.y() - candidate) / sample_resolution) + 1;
      for (int i=0; i<=num_v_samples; ++i) {
        double v = candidate + (g.y() - candidate) * i / num_v_samples;
        double u = g.x();
        auto [wx, wy] = toWorld(u,v);
        unsigned int mx,my;
        if (!costmap_->worldToMap(wx,wy,mx,my) || costmap_->getCost(mx,my)>0) {
          path_clear = false; break;
        }
      }
      if (!path_clear) continue;
      double dist = std::abs(candidate - s.y());
      if (dist < min_dist) {
        min_dist = dist; best_v = candidate;
      }
    }
  }
  return best_v;
}


/// -------------------------
/// 최종 경로 생성 (모드 스위칭)
/// -------------------------
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (use_astar_ && astar_planner_) {
    return astar_planner_->createPlan(start, goal);
  }

  // 기존 StraightLine 코드 전체
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = global_frame_;
  path_msg.header.stamp = node_->now();
  path_msg.poses.clear();

  if (!costmap_) {
    RCLCPP_ERROR(node_->get_logger(), "Costmap not set");
    return path_msg;
  }

  // 회전 좌표계 정의 (latest_yaw_ 기반)
  double dir_x = std::cos(latest_yaw_);
  double dir_y = std::sin(latest_yaw_);
  double ortho_x = -std::sin(latest_yaw_);
  double ortho_y =  std::cos(latest_yaw_);

  auto toLocal = [&](double wx, double wy) -> std::pair<double,double> {
    double u = wx * dir_x + wy * dir_y;
    double v = wx * ortho_x + wy * ortho_y;
    return {u, v};
  };

  auto toWorld = [&](double u, double v) -> std::pair<double,double> {
    double wx = u * dir_x + v * ortho_x;
    double wy = u * dir_y + v * ortho_y;
    return {wx, wy};
  };

  auto s_local = toLocal(start.pose.position.x, start.pose.position.y);
  auto g_local = toLocal(goal.pose.position.x, goal.pose.position.y);

  double resolution = interpolation_resolution_;
  std::vector<geometry_msgs::msg::PoseStamped> temp_path;

  // ===== X-first (항상 고정) =====
  double mid_u = calculateOptimalMidX(start, goal);
  if (std::isnan(mid_u)) {
    RCLCPP_WARN(node_->get_logger(), "No valid mid_x (mid_u) found.");
    return path_msg;
  }

  double s_u = s_local.first;
  double s_v = s_local.second;
  double g_v = g_local.second;
  double g_u = g_local.first;

  int n_points = std::max(2, int(std::abs(mid_u - s_u) / resolution));
  for (int i = 0; i <= n_points; ++i) {
    double u = s_u + (mid_u - s_u) * i / double(n_points);
    double v = s_v;
    auto [wx, wy] = toWorld(u, v);
    geometry_msgs::msg::PoseStamped pose = start;
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    double orient = (mid_u >= s_u) ? latest_yaw_ : (latest_yaw_ + M_PI);
    tf2::Quaternion qq; qq.setRPY(0,0,orient);
    pose.pose.orientation = tf2::toMsg(qq);
    temp_path.push_back(pose);
  }

  n_points = std::max(2, int(std::abs(g_v - s_v) / resolution));
  for (int i = 1; i <= n_points; ++i) {
    double v = s_v + (g_v - s_v) * i / double(n_points);
    double u = mid_u;
    auto [wx, wy] = toWorld(u, v);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    double orient = latest_yaw_ + M_PI_2;
    tf2::Quaternion qq; qq.setRPY(0,0,orient);
    pose.pose.orientation = tf2::toMsg(qq);
    temp_path.push_back(pose);
  }

  n_points = std::max(2, int(std::abs(g_u - mid_u) / resolution));
  for (int i = 1; i <= n_points; ++i) {
    double u = mid_u + (g_u - mid_u) * i / double(n_points);
    double v = g_v;
    auto [wx, wy] = toWorld(u, v);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    double orient = (g_u >= mid_u) ? latest_yaw_ : (latest_yaw_ + M_PI);
    tf2::Quaternion qq; qq.setRPY(0,0,orient);
    pose.pose.orientation = tf2::toMsg(qq);
    temp_path.push_back(pose);
  }

  path_msg.poses = temp_path;
  RCLCPP_INFO(node_->get_logger(),
    "Final path generated with StraightLine strategy: %zu poses",
    path_msg.poses.size());

  return path_msg;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
