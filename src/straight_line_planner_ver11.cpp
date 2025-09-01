#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <queue>
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

static inline int idx(int x, int y, int w) { return y * w + x; }

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
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_weight", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".unknown_is_lethal", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".guide_weight", rclcpp::ParameterValue(1.0));  // 새 파라미터

  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);
  node_->get_parameter(name_ + ".cost_weight", cost_weight_);
  node_->get_parameter(name_ + ".unknown_is_lethal", unknown_is_lethal_);
  node_->get_parameter(name_ + ".guide_weight", guide_weight_);  // 가져오기
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}

double StraightLine::calculateOptimalMidX(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  int num_divisions = 10;
  double best_mid_x = start.pose.position.x;
  double min_cost = std::numeric_limits<double>::max();

  double buffer = 1.5;
  double x_min = std::min(start.pose.position.x, goal.pose.position.x) - buffer;
  double x_max = std::max(start.pose.position.x, goal.pose.position.x) + buffer;

  for (int i = 0; i < num_divisions; ++i) {
    double candidate_x = x_min + i / double(num_divisions - 1) * (x_max - x_min);
    int num_points = 20;
    double cost = 0.0;
    for (int j = 0; j <= num_points; ++j) {
      double ratio = j / double(num_points);
      double y = start.pose.position.y + ratio * (goal.pose.position.y - start.pose.position.y);
      unsigned int mx, my;
      if (costmap_->worldToMap(candidate_x, y, mx, my)) {
        cost += costmap_->getCost(mx, my);
      } else {
        cost += 255;
      }
    }

    if (cost < min_cost - 1e-6) {
      min_cost = cost;
      best_mid_x = candidate_x;
    } else if (std::abs(cost - min_cost) < 1e-6) {
      double mid_target_x = (start.pose.position.x + goal.pose.position.x) / 2.0;
      if (std::abs(candidate_x - mid_target_x) < std::abs(best_mid_x - mid_target_x)) {
        best_mid_x = candidate_x;
      }
    }
  }

  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_->lookupTransform(global_frame_, "base_link", tf2::TimePointZero);
    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;

    // y 위치가 목표 y와 가까우면 mid_x를 로봇 x로 고정
    if (std::abs(robot_y - goal.pose.position.y) < 0.50) {
      RCLCPP_WARN(node_->get_logger(),
                  "Robot y is close to goal y (%.2f), fixing mid_x to robot x (%.2f)",
                  robot_y, robot_x);
      best_mid_x = robot_x;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
                "Could not get robot pose for mid_x adjustment: %s", ex.what());
  }

  return best_mid_x;
}

// ---------------------------------------------
// 최종 경로 생성 (ㄷ자 + A*)
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

  unsigned int sx_u, sy_u, gx_u, gy_u;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx_u, sy_u) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx_u, gy_u)) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal is out of map bounds");
    return path_msg;
  }

  int sx = static_cast<int>(sx_u);
  int sy = static_cast<int>(sy_u);
  int gx = static_cast<int>(gx_u);
  int gy = static_cast<int>(gy_u);
  int W = static_cast<int>(costmap_->getSizeInCellsX());
  int H = static_cast<int>(costmap_->getSizeInCellsY());

  // ---------------------------------------------
  // ㄷ자 mid_x 계산
  // ---------------------------------------------
  double mid_x = calculateOptimalMidX(start, goal);

  geometry_msgs::msg::PoseStamped pose_start = start;
  path_msg.poses.push_back(pose_start);

  geometry_msgs::msg::PoseStamped pose1;
  pose1.pose.position.x = mid_x;
  pose1.pose.position.y = start.pose.position.y;
  pose1.pose.position.z = 0.0;
  tf2::Quaternion q1; q1.setRPY(0,0,(goal.pose.position.x>=start.pose.position.x)?0.0:M_PI);
  pose1.pose.orientation = tf2::toMsg(q1);
  path_msg.poses.push_back(pose1);

  geometry_msgs::msg::PoseStamped pose2;
  pose2.pose.position.x = mid_x;
  pose2.pose.position.y = goal.pose.position.y;
  pose2.pose.position.z = 0.0;
  tf2::Quaternion q2; q2.setRPY(0,0,M_PI_2);
  pose2.pose.orientation = tf2::toMsg(q2);
  path_msg.poses.push_back(pose2);

  geometry_msgs::msg::PoseStamped pose_goal = goal;
  tf2::Quaternion q_goal; 
  q_goal.setRPY(0,0,std::atan2(goal.pose.position.y - pose2.pose.position.y,
                               goal.pose.position.x - pose2.pose.position.x));
  pose_goal.pose.orientation = tf2::toMsg(q_goal);
  path_msg.poses.push_back(pose_goal);

  // ---------------------------------------------
  // guide_path: ㄷ자 포인트를 셀 단위로 변환
  // ---------------------------------------------
  std::vector<std::pair<int,int>> guide_path;
  for(auto & pose: path_msg.poses) {
    unsigned int mx,my;
    if(costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
      guide_path.emplace_back(mx,my);
  }

  // ---------------------------------------------
  // A* 경로 탐색
  // ---------------------------------------------
  struct PQItem { std::pair<int,int> coord; double cost; bool operator<(const PQItem & o) const { return cost > o.cost; } };
  std::priority_queue<PQItem> open;
  std::vector<double> g(W*H,std::numeric_limits<double>::infinity());
  std::vector<int> parent(W*H,-1);
  std::vector<uint8_t> closed(W*H,0);

  auto hfun = [&](int x,int y){ return 0.0; };  // 휴리스틱 제거
  auto guidePenalty = [&](int x,int y){
    double min_dist = std::numeric_limits<double>::max();
    for(auto & p: guide_path){
      double dx = x-p.first, dy = y-p.second;
      double d = std::sqrt(dx*dx + dy*dy);
      if(d<min_dist) min_dist=d;
    }
    return min_dist * guide_weight_;  // 파라미터화
  };

  g[idx(sx,sy,W)] = 0.0;
  open.push(PQItem{{sx,sy}, hfun(sx,sy)});

  const int dx4[4] = {1,-1,0,0};
  const int dy4[4] = {0,0,1,-1};

  bool found=false;
  while(!open.empty()){
    auto u = open.top(); open.pop();
    int ux=u.coord.first, uy=u.coord.second;
    int ui = idx(ux,uy,W);
    if(closed[ui]) continue;
    closed[ui]=1;

    if(ux==gx && uy==gy){found=true; break;}

    for(int k=0;k<4;k++){
      int vx=ux+dx4[k], vy=uy+dy4[k];
      if(vx<0||vy<0||vx>=W||vy>=H) continue;
      if(costmap_->getCost(vx,vy)>=nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      int vi = idx(vx,vy,W);
      if(closed[vi]) continue;
      double tentative = g[ui]+1.0+guidePenalty(vx,vy);
      if(tentative < g[vi]){
        g[vi]=tentative;
        parent[vi]=ui;
        open.push(PQItem{{vx,vy}, tentative+hfun(vx,vy)});
      }
    }
  }

  // ---------------------------------------------
  // A* 결과 복원 및 world 좌표 변환
  // ---------------------------------------------
  std::vector<std::pair<int,int>> final_path;
  if(found){
    for(int v=idx(gx,gy,W); v!=-1; v=parent[v])
      final_path.emplace_back(v%W, v/W);
    std::reverse(final_path.begin(), final_path.end());
  } else {
    RCLCPP_WARN(node_->get_logger(), "A* failed, using guide only");
    final_path = guide_path;
  }

  nav_msgs::msg::Path final_path_msg;
  final_path_msg.header = path_msg.header;
  for(auto & p: final_path){
    double wx,wy;
    costmap_->mapToWorld(p.first,p.second,wx,wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = final_path_msg.header;
    pose.pose.position.x=wx;
    pose.pose.position.y=wy;
    pose.pose.position.z=0.0;
    final_path_msg.poses.push_back(pose);
  }

  RCLCPP_WARN(node_->get_logger(), "Final D-Shaped A* Plan DONE: %zu poses", final_path_msg.poses.size());
  return final_path_msg;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
