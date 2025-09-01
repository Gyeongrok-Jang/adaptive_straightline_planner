#include <cmath>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // use modern header
#include "pluginlib/class_list_macros.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

//---------------------------------------------------------
// Helpers
//---------------------------------------------------------
static inline int idx(int x, int y, int w) { return y * w + x; }

//---------------------------------------------------------
// Life-cycle hooks
//---------------------------------------------------------
void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // Parameters
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".cost_weight", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".unknown_is_lethal", rclcpp::ParameterValue(false));

  node_->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);
  node_->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);
  node_->get_parameter(name_ + ".cost_weight", cost_weight_);
  node_->get_parameter(name_ + ".unknown_is_lethal", unknown_is_lethal_);
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}

//---------------------------------------------------------
// A* over costmap grid
//---------------------------------------------------------
nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = global_frame_;
  path_msg.header.stamp = node_->now();

  if (!costmap_) {
    RCLCPP_ERROR(node_->get_logger(), "Costmap not set");
    return path_msg;
  }

  // Map start/goal to grid
  unsigned int sx_u, sy_u, gx_u, gy_u;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx_u, sy_u) ||
      !costmap_->worldToMap(goal.pose.position.x,  goal.pose.position.y,  gx_u, gy_u))
  {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal is out of map bounds");
    return path_msg;
  }
  const int sx = static_cast<int>(sx_u);
  const int sy = static_cast<int>(sy_u);
  const int gx = static_cast<int>(gx_u);
  const int gy = static_cast<int>(gy_u);

  const int W = static_cast<int>(costmap_->getSizeInCellsX());
  const int H = static_cast<int>(costmap_->getSizeInCellsY());
  const int N = W * H;

  // 4 or 8 connectivity
  // const int dx8[8] = { 1,-1, 0, 0, 1, 1,-1,-1};
  // const int dy8[8] = { 0, 0, 1,-1, 1,-1, 1,-1};
  // const int K = allow_diagonal_ ? 8 : 4;
  const int dx8[4] = { 1, -1, 0, 0 };
  const int dy8[4] = { 0, 0, 1, -1 };
  const int K = 4;

  // Buffers
  std::vector<double> g(N, std::numeric_limits<double>::infinity());
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);

  auto inBounds = [&](int x, int y){ return x >= 0 && y >= 0 && x < W && y < H; };
  auto isBlocked = [&](int x, int y){
    unsigned char c = costmap_->getCost(x, y);
    if (c == nav2_costmap_2d::NO_INFORMATION)
      return unknown_is_lethal_;
    return c >= nav2_costmap_2d::LETHAL_OBSTACLE;
  };

  auto hfun = [&](int x, int y){
    // Octile distance heuristic (admissible for 8-connected grids)
    double dx = std::abs(x - gx);
    double dy = std::abs(y - gy);
    double h = (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
    return heuristic_weight_ * h;
  };

  // Priority queue item declared in header: struct PQItem { std::pair<int,int> coord; double cost; ... }
  std::priority_queue<PQItem> open;

  const int s_idx = idx(sx, sy, W);
  g[s_idx] = 0.0;
  open.push(PQItem{{sx, sy}, hfun(sx, sy)});

  auto cell_cost_penalty = [&](int x, int y){
    unsigned char c = costmap_->getCost(x, y);
    if (c == nav2_costmap_2d::NO_INFORMATION)
      return unknown_is_lethal_ ? std::numeric_limits<double>::infinity() : 0.0;
    if (c >= nav2_costmap_2d::LETHAL_OBSTACLE)
      return std::numeric_limits<double>::infinity();
    // Scale [0,252] -> [0,1], then weight
    return (static_cast<double>(c) / 252.0) * cost_weight_;
  };

  bool found = false;
  while (!open.empty()) {
    const auto u_coord = open.top().coord; open.pop();
    const int ux = u_coord.first;
    const int uy = u_coord.second;
    const int u = idx(ux, uy, W);

    if (closed[u]) continue;
    closed[u] = 1;

    if (ux == gx && uy == gy) { found = true; break; }

    for (int k = 0; k < K; ++k) {
      const int vx = ux + dx8[k];
      const int vy = uy + dy8[k];
      if (!inBounds(vx, vy) || isBlocked(vx, vy)) continue;

      const int v = idx(vx, vy, W);
      if (closed[v]) continue;

      const bool diag = (k >= 4);
      const double step = diag ? std::sqrt(2.0) : 1.0;
      const double penalty = cell_cost_penalty(vx, vy);
      if (!std::isfinite(penalty)) continue; // lethal or unknown lethal

      const double tentative = g[u] + step + penalty;
      if (tentative < g[v]) {
        g[v] = tentative;
        parent[v] = u;
        const double fscore = tentative + hfun(vx, vy);
        open.push(PQItem{{vx, vy}, fscore});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(node_->get_logger(), "A* failed to find a path");
    return path_msg;
  }

  // Reconstruct
  std::vector<std::pair<int,int>> cells;
  for (int v = idx(gx, gy, W); v != -1; v = parent[v]) {
    cells.emplace_back(v % W, v / W);
  }
  std::reverse(cells.begin(), cells.end());

  // Convert to world poses with heading
  path_msg.poses.reserve(cells.size());
  for (size_t i = 0; i < cells.size(); ++i) {
    double wx, wy;
    costmap_->mapToWorld(cells[i].first, cells[i].second, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < cells.size()) {
      double wx2, wy2;
      costmap_->mapToWorld(cells[i+1].first, cells[i+1].second, wx2, wy2);
      yaw = std::atan2(wy2 - wy, wx2 - wx);
    } else {
      yaw = std::atan2(goal.pose.position.y - wy, goal.pose.position.x - wx);
    }
    tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    path_msg.poses.push_back(pose);
  }

  RCLCPP_INFO(node_->get_logger(), "A* plan generated: %zu poses", path_msg.poses.size());
  return path_msg;
}

//---------------------------------------------------------
// Collision check (kept for potential external use)
//---------------------------------------------------------
bool StraightLine::checkCollision(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my))
    return true;
  unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION)
    return unknown_is_lethal_;
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

} // namespace nav2_straightline_planner

PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
