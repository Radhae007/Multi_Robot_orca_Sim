#ifndef NAV2_ORCA_PLANNER__ORCA_LOCAL_PLANNER_HPP_
#define NAV2_ORCA_PLANNER__ORCA_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

#include "turtlebot3_multi_robot/msg/robot_state.hpp"

namespace nav2_orca_planner
{

struct AgentState
{
  std::string id;
  double px{0.0};
  double py{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.3};
  rclcpp::Time last_updated;
};

struct LineConstraint
{
  double point_x{0.0};
  double point_y{0.0};
  double dir_x{0.0};  // Direction vector (parallel to line)
  double dir_y{0.0};
};

class OrcaLocalPlanner : public nav2_core::Controller
{
public:
  OrcaLocalPlanner() = default;
  ~OrcaLocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void p2pStateCallback(const turtlebot3_multi_robot::msg::RobotState::SharedPtr msg);

  bool computeLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::Point & lookahead_pt);

  void constructOrcaLines(
    double host_px, double host_py, double host_vx, double host_vy,
    std::vector<LineConstraint> & out_orca_lines);

  bool solve2DLinearProgram(
    const std::vector<LineConstraint> & lines,
    double max_speed,
    double pref_vx, double pref_vy,
    double & opt_vx, double & opt_vy);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  std::string robot_id_;

  nav_msgs::msg::Path global_plan_;
  std::mutex plan_mutex_;
  std::mutex peer_mutex_;

  rclcpp::Subscription<turtlebot3_multi_robot::msg::RobotState>::SharedPtr p2p_sub_;
  std::map<std::string, AgentState> peer_agents_;

  // Parameters
  double max_vel_x_{0.26};
  double max_vel_theta_{1.0};
  double acc_lim_x_{2.5};
  double acc_lim_theta_{3.2};
  double lookahead_dist_{0.5};
  double time_horizon_{2.5};
  double neighbor_dist_{3.0};
  double agent_radius_{0.22};
  double safety_margin_{0.1};

  // State filtering for smooth transitions
  double prev_cmd_v_{0.0};
  double prev_cmd_w_{0.0};
  rclcpp::Time prev_time_;
};

}  // namespace nav2_orca_planner

#endif  // NAV2_ORCA_PLANNER__ORCA_LOCAL_PLANNER_HPP_
