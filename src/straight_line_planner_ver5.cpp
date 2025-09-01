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
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // declare parameters
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".cost_weight", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".unknown_is_lethal", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".inflation_radius", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".inflation_penalty", rclcpp::ParameterValue(20.0));

  // get parameters
  node_->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);
  node_->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);
  node_->get_parameter(name_ + ".cost_weight", cost_weight_);
  node_->get_parameter(name_ + ".unknown_is_lethal", unknown_is_lethal_);
  node_->get_parameter(name_ + ".inflation_radius", inflation_radius_);
  node_->get_parameter(name_ + ".inflation_penalty", inflation_penalty_);
}

void StraightLine::cleanup() {}
void StraightLine::activate() {}
void StraightLine::deactivate() {}

bool StraightLine::checkCollision(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return true;
  unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION)
    return unknown_is_lethal_;
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

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

  //---------------------------------------------------------
  // 1. Generate guide (Manhattan) path
  //---------------------------------------------------------
  std::vector<std::pair<int,int>> guide_path;
  int x = sx, y = sy;
  while (x != gx || y != gy) {
    guide_path.emplace_back(x, y);
    if (x != gx) x += (gx > x) ? 1 : -1;
    else if (y != gy) y += (gy > y) ? 1 : -1;
  }
  guide_path.emplace_back(gx, gy);

  //---------------------------------------------------------
  // 2. Helper lambdas
  //---------------------------------------------------------
  auto inBounds = [&](int x, int y){ return x>=0 && y>=0 && x<W && y<H; };
  auto isBlocked = [&](int x, int y){
    unsigned char c = costmap_->getCost(x, y);
    if (c == nav2_costmap_2d::NO_INFORMATION) return unknown_is_lethal_;
    return c >= nav2_costmap_2d::LETHAL_OBSTACLE;
  };
  auto guidePenalty = [&](int x, int y){
    double min_dist = std::numeric_limits<double>::max();
    for (auto & p : guide_path){
      double dx = x - p.first;
      double dy = y - p.second;
      double d = std::sqrt(dx*dx + dy*dy);
      if (d < min_dist) min_dist = d;
    }
    return min_dist * cost_weight_; // guide 가중치
  };

  //---------------------------------------------------------
  // 3. Setup A* buffers
  //---------------------------------------------------------
  std::vector<double> g(W*H, std::numeric_limits<double>::infinity());
  std::vector<int> parent(W*H, -1);
  std::vector<uint8_t> closed(W*H, 0);

  auto hfun = [&](int x,int y){
    double dx = std::abs(x - gx);
    double dy = std::abs(y - gy);
    return heuristic_weight_ * (dx + dy);
  };

  struct PQItem { std::pair<int,int> coord; double cost; bool operator<(const PQItem & o) const { return cost > o.cost; } };
  std::priority_queue<PQItem> open;

  g[idx(sx, sy, W)] = 0.0;
  open.push(PQItem{{sx, sy}, hfun(sx, sy)});

  const int dx4[4] = {1,-1,0,0};
  const int dy4[4] = {0,0,1,-1};

  //---------------------------------------------------------
  // 4. Run A* with guide + inflation penalty
  //---------------------------------------------------------
  bool found=false;
  while(!open.empty()){
    auto u=open.top(); open.pop();
    int ux=u.coord.first, uy=u.coord.second;
    int ui=idx(ux, uy, W);
    if(closed[ui]) continue;
    closed[ui]=1;

    if(ux==gx && uy==gy){found=true; break;}

    for(int k=0;k<4;k++){
      int vx=ux+dx4[k], vy=uy+dy4[k];
      if(!inBounds(vx,vy) || isBlocked(vx,vy)) continue;
      int vi=idx(vx, vy, W);
      if(closed[vi]) continue;

      // 장애물 근처일수록 inflation_penalty 적용
      unsigned char c = costmap_->getCost(vx, vy);
      double inflation_cost = 0.0;
      if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        inflation_cost = inflation_penalty_;

      double penalty = guidePenalty(vx, vy) + inflation_cost;
      double tentative = g[ui] + 1.0 + penalty;
      if(tentative < g[vi]){
        g[vi]=tentative;
        parent[vi]=ui;
        open.push(PQItem{{vx,vy}, tentative + hfun(vx,vy)});
      }
    }
  }

  //---------------------------------------------------------
  // 5. Reconstruct path
  //---------------------------------------------------------
  std::vector<std::pair<int,int>> final_path;
  if(found){
    for(int v=idx(gx, gy, W); v!=-1; v=parent[v])
      final_path.emplace_back(v%W, v/W);
    std::reverse(final_path.begin(), final_path.end());
  } else {
    RCLCPP_WARN(node_->get_logger(), "A* failed, using guide path only");
    final_path = guide_path;
  }

  //---------------------------------------------------------
  // 6. Convert to world
  //---------------------------------------------------------
  for(auto & p : final_path){
    double wx, wy;
    costmap_->mapToWorld(p.first, p.second, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x=wx;
    pose.pose.position.y=wy;
    pose.pose.position.z=0.0;
    path_msg.poses.push_back(pose);
  }

  RCLCPP_INFO(node_->get_logger(), "A* path generated along guide: %zu poses", path_msg.poses.size());
  return path_msg;
}

} // namespace nav2_straightline_planner
