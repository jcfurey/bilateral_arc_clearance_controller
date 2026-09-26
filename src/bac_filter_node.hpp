/**
 * @file bac_filter_node.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief ROS 2 evaluation node for bac_core: cmd_vel filter driven by a laser scan
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Not installed: the node is used through its component
 * (bac::BacFilterNode) or the bac_filter_node executable. The declaration is
 * a header only so the adapter test can instantiate the node in-process.
 */

#pragma once
#ifndef BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_FILTER_NODE_HPP_
#define BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_FILTER_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8.hpp"

namespace bac
{

/// The default of `enable_stamped_cmd_vel`: the message type Nav2's own
/// cmd_vel publishers use on the distribution this node is built for.
/// geometry_msgs/Twist through Jazzy, TwistStamped from Kilted (rclcpp 29) on.
bool defaultStampedCmdVel();

/**
 * cmd_vel filter driven by a laser scan.
 *
 * Topics:
 *   in:  cmd_vel_in  (geometry_msgs/Twist or TwistStamped) upper-level command
 *        scan        (sensor_msgs/LaserScan)                obstacle points
 *        odom        (nav_msgs/Odometry)                    velocity feedback
 *   out: cmd_vel_out (geometry_msgs/Twist or TwistStamped) collision-shaped command
 *        avoid_status (std_msgs/Int8)                       0=CLEAR 1=AVOIDING 2=STOP
 *
 * `enable_stamped_cmd_vel` selects the command type for both cmd_vel topics.
 * The node ticks every `control_period` on the node clock, so it follows
 * simulated time under `use_sim_time`. While status==CLEAR the input command
 * is passed through (angular-rate limited); stale scan or odometry forces
 * zero output.
 */
class BacFilterNode : public rclcpp::Node
{
public:
  explicit BacFilterNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /// The tick period the timer was created with [s]
  double controlPeriod() const { return control_period_; }

  /// Whether both cmd_vel topics carry geometry_msgs/TwistStamped
  bool stampedCmdVel() const { return stamped_; }

private:
  float declareFloat(const std::string &name, float default_value);
  void  onCommand(const geometry_msgs::msg::Twist &twist, const std::string &frame_id);
  void  tick();
  void  publish(const Twist2D &command, Status status);

  rcl_interfaces::msg::SetParametersResult
  onSetParameters(const std::vector<rclcpp::Parameter> &parameters) const;

  BacCore core_;

  std::mutex           mutex_;
  Twist2D              command_;
  std::string          command_frame_;
  Twist2D              current_;
  std::vector<Point2D> points_;
  rclcpp::Time         last_scan_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time         last_cmd_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time         last_odom_time_{ 0, 0, RCL_ROS_TIME };
  int                  valid_rays_ = 0;

  float  sensor_x_ = 0.0f, sensor_y_ = 0.0f, sensor_yaw_ = 0.0f;
  float  scan_timeout_        = 0.5f;
  float  cmd_timeout_         = 0.5f;
  float  odom_timeout_        = 0.5f;
  int    scan_min_points_     = 10;
  bool   scan_inf_is_valid_   = true;
  float  base_v_max_          = 0.4f;
  float  max_range_           = 10.0f;  // [m] scan projection bound, fixed at construction
  float  virtual_path_length_ = 3.0f;
  double control_period_      = 0.05;
  bool   stamped_             = false;

  /// Every parameter this node reads at construction; changes are refused.
  std::vector<std::string> parameter_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_stamped_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr                 status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_FILTER_NODE_HPP_
