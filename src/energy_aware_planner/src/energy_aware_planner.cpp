#include <cmath>
#include <algorithm>
#include "energy_aware_planner/energy_aware_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(energy_aware_planner::EnergyAwarePlanner, nav2_core::GlobalPlanner)

namespace energy_aware_planner
{

void EnergyAwarePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  (void)tf;
  auto node = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  node->get_parameter_or(name + ".beta_crit", beta_crit_, 0.2);
  node->get_parameter_or(name + ".k_sigmoid", k_sigmoid_, 10.0);

  battery_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    "/battery", 10, std::bind(&EnergyAwarePlanner::batteryCallback, this, std::placeholders::_1));

  loadZones();
  RCLCPP_INFO(node->get_logger(), "EnergyAwarePlanner Configured.");

  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("energy_zones", 10);
}

void EnergyAwarePlanner::loadZones() {
  zones_.clear();

  // ZONE A: The "Expensive Shortcut" (Top Lane)
  ZoneDefinition shortcut;
  shortcut.name = "heavy_carpet";
  shortcut.min_mx = 90;   shortcut.max_mx = 130; 
  shortcut.min_my = 140; shortcut.max_my = 180;
  shortcut.energy_usage_factor = 25.0; // Extremely high friction
  zones_.push_back(shortcut);

  // ZONE B: The "Compromise" (Middle Lane)
  ZoneDefinition middle;
  middle.name = "standard_rug";
  middle.min_mx = 90;   middle.max_mx = 130; 
  middle.min_my = 80;  middle.max_my = 120;
  middle.energy_usage_factor = 8.0;  // Moderate friction
  zones_.push_back(middle);

  // ZONE C: The "Efficient Detour" (Bottom Lane)
  // No zone definition needed; default mu = 1.0 (Smooth Tile)
}

void EnergyAwarePlanner::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_battery_ = msg->data / 100.0;
}

double EnergyAwarePlanner::calculateAlpha(double beta)
{
  return 1.0 / (1.0 + std::exp(k_sigmoid_ * (beta - beta_crit_)));
}

double EnergyAwarePlanner::euclideanDist(unsigned int idx1, unsigned int idx2)
{
  unsigned int mx1, my1, mx2, my2;
  costmap_->indexToCells(idx1, mx1, my1);
  costmap_->indexToCells(idx2, mx2, my2);
  return std::hypot(static_cast<double>(mx1) - mx2, static_cast<double>(my1) - my2);
}

std::vector<unsigned int> EnergyAwarePlanner::getNeighbors(unsigned int index) {
  std::vector<unsigned int> neighbors;
  int nx = static_cast<int>(costmap_->getSizeInCellsX());
  int ny = static_cast<int>(costmap_->getSizeInCellsY());
  unsigned int mx_u, my_u;
  costmap_->indexToCells(index, mx_u, my_u);
  
  int mx = static_cast<int>(mx_u);
  int my = static_cast<int>(my_u);

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      int nmx = mx + dx;
      int nmy = my + dy;
      // Fixed pixel underflow check
      if (nmx >= 0 && nmx < nx && nmy >= 0 && nmy < ny) {
        neighbors.push_back(costmap_->getIndex(static_cast<unsigned int>(nmx), static_cast<unsigned int>(nmy)));
      }
    }
  }
  return neighbors;
}

double EnergyAwarePlanner::getTraversalCost(unsigned int from_idx, unsigned int to_idx, double alpha) {
  if (costmap_->getCost(to_idx) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return -1.0;

  double d = euclideanDist(from_idx, to_idx);
  double mu = 1.0;

  unsigned int mx, my;
  costmap_->indexToCells(to_idx, mx, my);
  for (const auto& zone : zones_) {
    if (mx >= zone.min_mx && mx <= zone.max_mx && my >= zone.min_my && my <= zone.max_my) {
      mu = zone.energy_usage_factor;
      break;
    }
  }
  return (1.0 - alpha) * d + alpha * (mu * d);
}

nav_msgs::msg::Path EnergyAwarePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  publishZoneMarker();
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = rclcpp::Clock().now();
  global_path.header.frame_id = global_frame_;

  unsigned int mx, my, start_idx, goal_idx;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) return global_path;
  start_idx = costmap_->getIndex(mx, my);
  
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) return global_path;
  goal_idx = costmap_->getIndex(mx, my);

  // Safety Check: Avoid planning if goal is inside a wall
  if (costmap_->getCost(goal_idx) >= 253) {
      RCLCPP_WARN(rclcpp::get_logger(name_), "Goal is in an obstacle!");
      return global_path;
  }

  double alpha = calculateAlpha(current_battery_);
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
  std::unordered_map<unsigned int, double> g_costs;
  std::unordered_map<unsigned int, unsigned int> came_from;

  open_set.push({start_idx, 0.0, euclideanDist(start_idx, goal_idx), start_idx});
  g_costs[start_idx] = 0.0;

  while (!open_set.empty()) {
    Node current = open_set.top();
    open_set.pop();

    if (current.index == goal_idx) {
      unsigned int curr = goal_idx;
      // --- DECLARATIONS FOR STEP 3 ---
      double total_distance = 0.0;
      double total_energy_weighted_cost = 0.0;
      // -------------------------------
      while (curr != start_idx) {
        geometry_msgs::msg::PoseStamped pose;
        unsigned int cmx, cmy;
        costmap_->indexToCells(curr, cmx, cmy);
        double wx, wy;
        costmap_->mapToWorld(cmx, cmy, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;
        global_path.poses.push_back(pose);
        // --- NEW LOGIC FOR STEP 3 ---
        unsigned int parent = came_from[curr];
        double step_dist = euclideanDist(parent, curr) * costmap_->getResolution();
        total_distance += step_dist;
        
        // Calculate the mu (friction) of this specific step
        double mu = 1.0;
        for (const auto& zone : zones_) {
            if (cmx >= zone.min_mx && cmx <= zone.max_mx && cmy >= zone.min_my && cmy <= zone.max_my) {
                mu = zone.energy_usage_factor;
                break;
            }
        }
        total_energy_weighted_cost += (mu * step_dist);
        // -----------------------------

        curr = parent;
      }
      std::reverse(global_path.poses.begin(), global_path.poses.end());

      // --- THE "THESIS DEFENSE" LOG ---
      
      bool goal_changed = (std::abs(goal.pose.position.x - last_goal_pos_.x) > 0.1 || 
                     std::abs(goal.pose.position.y - last_goal_pos_.y) > 0.1);
      bool battery_changed = (std::abs(current_battery_ - last_reported_battery_) > 0.01);

      if (goal_changed || battery_changed) {
          RCLCPP_INFO(rclcpp::get_logger(name_), "--- Path Optimization Report ---");
          RCLCPP_INFO(rclcpp::get_logger(name_), "Battery: %.1f%% | Alpha: %.3f", current_battery_ * 100.0, alpha);
          RCLCPP_INFO(rclcpp::get_logger(name_), "Actual Distance: %.2f meters", total_distance);
          RCLCPP_INFO(rclcpp::get_logger(name_), "Energy-Weighted Cost: %.2f", total_energy_weighted_cost);
          
          if (alpha < 0.1) {
              RCLCPP_INFO(rclcpp::get_logger(name_), "Mode: PERFORMANCE (Shortest Path)");
          } else if (alpha > 0.6) {
              RCLCPP_INFO(rclcpp::get_logger(name_), "Mode: CONSERVATION (Energy Optimal)");
          } else {
              RCLCPP_INFO(rclcpp::get_logger(name_), "Mode: BALANCED (Optimization Trade-off)");
          }
          RCLCPP_INFO(rclcpp::get_logger(name_), "---------------------------------");

          // Update the "last" states
          last_reported_battery_ = current_battery_;
          last_goal_pos_ = goal.pose.position;
          }


        
      return global_path;
    }

    for (unsigned int neighbor : getNeighbors(current.index)) {
      double step_cost = getTraversalCost(current.index, neighbor, alpha);
      if (step_cost < 0) continue;
      double tentative_g = g_costs[current.index] + step_cost;
      if (g_costs.find(neighbor) == g_costs.end() || tentative_g < g_costs[neighbor]) {
        came_from[neighbor] = current.index;
        g_costs[neighbor] = tentative_g;
        open_set.push({neighbor, tentative_g, euclideanDist(neighbor, goal_idx), current.index});
      }
    }
  }
  return global_path;
}

void EnergyAwarePlanner::publishZoneMarker() {
  auto now = rclcpp::Clock().now();

  for (size_t i = 0; i < zones_.size(); ++i) {
    // 1. PROFESSIONAL SURFACE TEXTURE
    visualization_msgs::msg::Marker surface;
    surface.header.frame_id = global_frame_;
    surface.header.stamp = now;
    surface.ns = "warehouse_zones";
    surface.id = static_cast<int>(i);
    surface.type = visualization_msgs::msg::Marker::CUBE;
    
    double x_min, y_min, x_max, y_max;
    costmap_->mapToWorld(zones_[i].min_mx, zones_[i].min_my, x_min, y_min);
    costmap_->mapToWorld(zones_[i].max_mx, zones_[i].max_my, x_max, y_max);

    surface.pose.position.x = (x_min + x_max) / 2.0;
    surface.pose.position.y = (y_min + y_max) / 2.0;
    surface.pose.position.z = 0.005; // Flush with floor
    surface.scale.x = std::abs(x_max - x_min);
    surface.scale.y = std::abs(y_max - y_min);
    surface.scale.z = 0.001;

    // Use "Industrial" Colors (Semi-Transparent)
    if (zones_[i].energy_usage_factor > 15.0) {
      surface.color.r = 0.8; surface.color.g = 0.0; surface.color.b = 0.0; surface.color.a = 0.25; // Warning Red
    } else {
      surface.color.r = 0.8; surface.color.g = 0.5; surface.color.b = 0.0; surface.color.a = 0.25; // Caution Amber
    }
    marker_pub_->publish(surface);

    // 2. FIXED FLOATING LABELS
    visualization_msgs::msg::Marker label;
    label.header.frame_id = global_frame_;
    label.header.stamp = now;
    label.ns = "zone_descriptors";
    label.id = static_cast<int>(i + 100);
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.position.x = surface.pose.position.x;
    label.pose.position.y = surface.pose.position.y;
    label.pose.position.z = 1.2; // Float high above robot
    label.scale.z = 0.4; 
    label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 1.0;
    
    label.text = (zones_[i].energy_usage_factor > 15.0) ? "[ DANGER: DEBRIS ZONE ]" : "[ CAUTION: HIGH TRAFFIC ]";
    marker_pub_->publish(label);
  }
}

void EnergyAwarePlanner::cleanup() {}
void EnergyAwarePlanner::activate() {}
void EnergyAwarePlanner::deactivate() {}

}
