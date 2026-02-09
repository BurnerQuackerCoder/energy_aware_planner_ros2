#ifndef ENERGY_AWARE_PLANNER__ENERGY_AWARE_PLANNER_HPP_
#define ENERGY_AWARE_PLANNER__ENERGY_AWARE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace energy_aware_planner
{

struct ZoneDefinition {
    std::string name;
    unsigned int min_mx, max_mx, min_my, max_my;
    double energy_usage_factor; 
};

struct Node {
    unsigned int index;
    double g_cost;
    double h_cost;
    unsigned int parent_index;

    double f_cost() const { return g_cost + h_cost; }
    bool operator>(const Node& other) const { return f_cost() > other.f_cost(); }
};

class EnergyAwarePlanner : public nav2_core::GlobalPlanner
{
public:
  EnergyAwarePlanner() = default;
  ~EnergyAwarePlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
  double calculateAlpha(double battery_level);
  std::vector<unsigned int> getNeighbors(unsigned int index);
  double getTraversalCost(unsigned int from_idx, unsigned int to_idx, double alpha);
  double euclideanDist(unsigned int idx1, unsigned int idx2);
  void loadZones();

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  void publishZoneMarker();
  
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  std::string global_frame_, name_;
  
  // Thesis Parameters
  double current_battery_ = 100.0;
  double beta_crit_ = 0.2; 
  double k_sigmoid_ = 10.0; 
  std::vector<ZoneDefinition> zones_;
  double last_reported_battery_ = -1.0;
  geometry_msgs::msg::Point last_goal_pos_;
};

}  // namespace energy_aware_planner

#endif  // ENERGY_AWARE_PLANNER__ENERGY_AWARE_PLANNER_HPP_

// #ifndef ENERGY_AWARE_PLANNER__ENERGY_AWARE_PLANNER_HPP_
// #define ENERGY_AWARE_PLANNER__ENERGY_AWARE_PLANNER_HPP_

// #include <memory>
// #include <string>
// #include <vector>
// #include <queue>
// #include <unordered_map>

// #include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_core/global_planner.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "nav2_util/lifecycle_node.hpp"
// #include "nav2_util/robot_utils.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "rclcpp/rclcpp.hpp"

// namespace energy_aware_planner
// {

// struct ZoneDefinition {
//     std::string name;
//     unsigned int min_mx, max_mx, min_my, max_my;
//     double energy_usage_factor; // Equivalent to friction coefficient mu in thesis
// };

// // Helper for A* search
// struct Node {
//     unsigned int index;
//     double g_cost;
//     double h_cost;
//     unsigned int parent_index;

//     double f_cost() const { return g_cost + h_cost; }
//     bool operator>(const Node& other) const { return f_cost() > other.f_cost(); }
// };

// class EnergyAwarePlanner : public nav2_core::GlobalPlanner
// {
// public:
//   EnergyAwarePlanner() = default;
//   ~EnergyAwarePlanner() = default;

//   void configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

//   void cleanup() override;
//   void activate() override;
//   void deactivate() override;

//   nav_msgs::msg::Path createPlan(
//     const geometry_msgs::msg::PoseStamped & start,
//     const geometry_msgs::msg::PoseStamped & goal) override;

// private:
//   void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
//   double calculateAlpha(double battery_level);
  
//   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
//   nav2_costmap_2d::Costmap2D * costmap_;
//   rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
//   std::string global_frame_, name_;
  
//   // Thesis Parameters
//   double current_battery_ = 100.0;
//   double beta_crit_ = 0.2;  // 20% critical threshold as per thesis
//   double k_sigmoid_ = 10.0; // Aggressiveness of transition
//   std::vector<ZoneDefinition> zones_;

//   // NEW: A* Helper Methods
//   std::vector<unsigned int> getNeighbors(unsigned int index);
//   double getTraversalCost(unsigned int from_idx, unsigned int to_idx, double alpha);
//   double euclideanDist(unsigned int idx1, unsigned int idx2);

//   // NEW: Energy parameters and Zone data
//   std::vector<ZoneDefinition> zones_;
//   void loadZones(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);
// };

// }  // namespace energy_aware_planner

// #endif  // ENERGY_AWARE_PLANNER__ENERGY_AWARE_PLANNER_HPP_