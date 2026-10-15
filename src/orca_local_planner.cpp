#include "nav2_orca_planner/orca_local_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include <algorithm>

namespace nav2_orca_planner
{

void OrcaLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock parent node in OrcaLocalPlanner::configure");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Extract namespace/robot_id
  std::string ns = node->get_namespace();
  while (!ns.empty() && ns.front() == '/') {
    ns.erase(ns.begin());
  }
  robot_id_ = ns.empty() ? "tb_host" : ns;

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.26));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".time_horizon", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".agent_radius", rclcpp::ParameterValue(0.22));

  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".time_horizon", time_horizon_);
  node->get_parameter(plugin_name_ + ".agent_radius", agent_radius_);

  // Global P2P subscriber for robot states across namespaces
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  p2p_sub_ = node->create_subscription<turtlebot3_multi_robot::msg::RobotState>(
    "/p2p/robot_states", qos,
    std::bind(&OrcaLocalPlanner::p2pStateCallback, this, std::placeholders::_1));

  prev_time_ = node->now();
  RCLCPP_INFO(
    node->get_logger(),
    "Configured OrcaLocalPlanner plugin '%s' for Robot ID '%s'",
    plugin_name_.c_str(), robot_id_.c_str());
}

void OrcaLocalPlanner::cleanup()
{
  p2p_sub_.reset();
}

void OrcaLocalPlanner::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("OrcaLocalPlanner"), "Activated ORCA Controller Plugin");
}

void OrcaLocalPlanner::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("OrcaLocalPlanner"), "Deactivated ORCA Controller Plugin");
}

void OrcaLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_ = path;
}

void OrcaLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_vel_x_ *= (speed_limit / 100.0);
  } else {
    max_vel_x_ = speed_limit;
  }
}

void OrcaLocalPlanner::p2pStateCallback(
  const turtlebot3_multi_robot::msg::RobotState::SharedPtr msg)
{
  if (msg->robot_id == robot_id_) {
    return;
  }
  std::lock_guard<std::mutex> lock(peer_mutex_);
  AgentState state;
  state.id = msg->robot_id;
  state.px = msg->pose.pose.position.x;
  state.py = msg->pose.pose.position.y;
  
  // Calculate global velocity components from twist & orientation
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  state.vx = msg->velocity.linear.x * std::cos(yaw) - msg->velocity.linear.y * std::sin(yaw);
  state.vy = msg->velocity.linear.x * std::sin(yaw) + msg->velocity.linear.y * std::cos(yaw);
  state.radius = agent_radius_;
  state.last_updated = rclcpp::Clock().now();

  peer_agents_[msg->robot_id] = state;
}

bool OrcaLocalPlanner::computeLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  geometry_msgs::msg::Point & lookahead_pt)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  if (global_plan_.poses.empty()) {
    return false;
  }

  double cur_x = pose.pose.position.x;
  double cur_y = pose.pose.position.y;

  for (const auto & pose_stamped : global_plan_.poses) {
    double dx = pose_stamped.pose.position.x - cur_x;
    double dy = pose_stamped.pose.position.y - cur_y;
    double dist = std::hypot(dx, dy);
    if (dist >= lookahead_dist_) {
      lookahead_pt = pose_stamped.pose.position;
      return true;
    }
  }

  lookahead_pt = global_plan_.poses.back().pose.position;
  return true;
}

void OrcaLocalPlanner::constructOrcaLines(
  double host_px, double host_py, double host_vx, double host_vy,
  std::vector<LineConstraint> & out_orca_lines)
{
  std::lock_guard<std::mutex> lock(peer_mutex_);
  rclcpp::Time now = rclcpp::Clock().now();

  for (const auto & pair : peer_agents_) {
    const AgentState & peer = pair.second;
    if ((now - peer.last_updated).seconds() > 2.0) {
      continue;
    }

    double rel_px = peer.px - host_px;
    double rel_py = peer.py - host_py;
    double dist_sq = rel_px * rel_px + rel_py * rel_py;
    double combined_radius = agent_radius_ + peer.radius + safety_margin_;
    double combined_radius_sq = combined_radius * combined_radius;

    if (dist_sq > neighbor_dist_ * neighbor_dist_) {
      continue;
    }

    double rel_vx = host_vx - peer.vx;
    double rel_vy = host_vy - peer.vy;

    LineConstraint line;
    double u_x = 0.0;
    double u_y = 0.0;

    if (dist_sq > combined_radius_sq) {
      // Vector from cutoff center to relative velocity
      double w_x = rel_vx - rel_px / time_horizon_;
      double w_y = rel_vy - rel_py / time_horizon_;
      double w_len_sq = w_x * w_x + w_y * w_y;

      double dot_product = w_x * rel_px + w_y * rel_py;

      if (dot_product < 0.0 && (dot_product * dot_product > combined_radius_sq * w_len_sq)) {
        // Cutoff circle projection
        double w_len = std::sqrt(w_len_sq);
        double unit_w_x = (w_len > 1e-6) ? w_x / w_len : 0.0;
        double unit_w_y = (w_len > 1e-6) ? w_y / w_len : 0.0;

        line.dir_x = unit_w_y;
        line.dir_y = -unit_w_x;

        u_x = (combined_radius / time_horizon_ - w_len) * unit_w_x;
        u_y = (combined_radius / time_horizon_ - w_len) * unit_w_y;
      } else {
        // Legs projection
        double dist = std::sqrt(dist_sq);
        double leg = std::sqrt(dist_sq - combined_radius_sq);

        if (rel_px * w_y - rel_py * w_x > 0.0) {
          // Left leg
          line.dir_x = (rel_px * leg - rel_py * combined_radius) / dist_sq;
          line.dir_y = (rel_py * leg + rel_px * combined_radius) / dist_sq;
        } else {
          // Right leg
          line.dir_x = -(rel_px * leg + rel_py * combined_radius) / dist_sq;
          line.dir_y = -(rel_py * leg - rel_px * combined_radius) / dist_sq;
        }

        double dot2 = rel_vx * line.dir_x + rel_vy * line.dir_y;
        u_x = dot2 * line.dir_x - rel_vx;
        u_y = dot2 * line.dir_y - rel_vy;
      }
    } else {
      // Colliding / overlapping state
      double dist = std::sqrt(dist_sq);
      double unit_rel_x = (dist > 1e-6) ? rel_px / dist : 1.0;
      double unit_rel_y = (dist > 1e-6) ? rel_py / dist : 0.0;

      line.dir_x = unit_rel_y;
      line.dir_y = -unit_rel_x;

      u_x = (combined_radius - dist) * unit_rel_x;
      u_y = (combined_radius - dist) * unit_rel_y;
    }

    // Reciprocal half-plane constraint (50% responsibility per robot)
    line.point_x = host_vx + 0.5 * u_x;
    line.point_y = host_vy + 0.5 * u_y;

    out_orca_lines.push_back(line);
  }
}

bool OrcaLocalPlanner::solve2DLinearProgram(
  const std::vector<LineConstraint> & lines,
  double max_speed,
  double pref_vx, double pref_vy,
  double & opt_vx, double & opt_vy)
{
  opt_vx = pref_vx;
  opt_vy = pref_vy;

  // Clip preferred velocity to max speed
  double speed = std::hypot(opt_vx, opt_vy);
  if (speed > max_speed && speed > 1e-6) {
    opt_vx = (opt_vx / speed) * max_speed;
    opt_vy = (opt_vy / speed) * max_speed;
  }

  // Iterate over ORCA constraint half-planes
  for (size_t i = 0; i < lines.size(); ++i) {
    const auto & line = lines[i];
    // Check constraint violation: (opt - point) x dir < 0
    if ((opt_vx - line.point_x) * line.dir_y - (opt_vy - line.point_y) * line.dir_x < 0.0) {
      // Project opt_v onto line constraint
      double dot = (opt_vx - line.point_x) * line.dir_x + (opt_vy - line.point_y) * line.dir_y;
      opt_vx = line.point_x + dot * line.dir_x;
      opt_vy = line.point_y + dot * line.dir_y;

      // Re-cap max speed
      speed = std::hypot(opt_vx, opt_vy);
      if (speed > max_speed && speed > 1e-6) {
        opt_vx = (opt_vx / speed) * max_speed;
        opt_vy = (opt_vy / speed) * max_speed;
      }
    }
  }
  return true;
}

geometry_msgs::msg::TwistStamped OrcaLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = node_.lock();
  rclcpp::Time now = node ? node->now() : rclcpp::Clock().now();

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = now;
  cmd_vel.header.frame_id = pose.header.frame_id;

  geometry_msgs::msg::Point lookahead_pt;
  if (!computeLookaheadPoint(pose, lookahead_pt)) {
    RCLCPP_WARN(rclcpp::get_logger("OrcaLocalPlanner"), "No valid global path lookahead point");
    return cmd_vel;
  }

  // Compute host orientation and current velocity in global frame
  double yaw = tf2::getYaw(pose.pose.orientation);
  double host_px = pose.pose.position.x;
  double host_py = pose.pose.position.y;

  double host_vx = velocity.linear.x * std::cos(yaw);
  double host_vy = velocity.linear.x * std::sin(yaw);

  // Preferred velocity vector pointing toward lookahead target
  double target_dx = lookahead_pt.x - host_px;
  double target_dy = lookahead_pt.y - host_py;
  double target_dist = std::hypot(target_dx, target_dy);

  double pref_vx = 0.0;
  double pref_vy = 0.0;
  if (target_dist > 0.05) {
    pref_vx = (target_dx / target_dist) * max_vel_x_;
    pref_vy = (target_dy / target_dist) * max_vel_x_;
  }

  // Construct ORCA constraints and solve
  std::vector<LineConstraint> lines;
  constructOrcaLines(host_px, host_py, host_vx, host_vy, lines);

  double opt_vx = 0.0;
  double opt_vy = 0.0;
  solve2DLinearProgram(lines, max_vel_x_, pref_vx, pref_vy, opt_vx, opt_vy);

  // Convert optimal global velocity (opt_vx, opt_vy) to local robot body frame (v_lin, w_ang)
  double opt_angle = std::atan2(opt_vy, opt_vx);
  double heading_err = std::atan2(std::sin(opt_angle - yaw), std::cos(opt_angle - yaw));

  double raw_linear = std::hypot(opt_vx, opt_vy) * std::cos(heading_err);
  double raw_angular = 2.0 * heading_err;

  raw_linear = std::clamp(raw_linear, 0.0, max_vel_x_);
  raw_angular = std::clamp(raw_angular, -max_vel_theta_, max_vel_theta_);

  // Apply smooth velocity transition (acceleration limit / EMA filter)
  double dt = (prev_time_.nanoseconds() > 0) ? (now - prev_time_).seconds() : 0.05;
  dt = std::clamp(dt, 0.01, 0.2);

  double max_dv = acc_lim_x_ * dt;
  double max_dw = acc_lim_theta_ * dt;

  double smooth_linear = prev_cmd_v_ + std::clamp(raw_linear - prev_cmd_v_, -max_dv, max_dv);
  double smooth_angular = prev_cmd_w_ + std::clamp(raw_angular - prev_cmd_w_, -max_dw, max_dw);

  cmd_vel.twist.linear.x = smooth_linear;
  cmd_vel.twist.angular.z = smooth_angular;

  prev_cmd_v_ = smooth_linear;
  prev_cmd_w_ = smooth_angular;
  prev_time_ = now;

  return cmd_vel;
}

}  // namespace nav2_orca_planner

PLUGINLIB_EXPORT_CLASS(nav2_orca_planner::OrcaLocalPlanner, nav2_core::Controller)
