/**
 * @file bac_controller.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Nav2 controller plugin wrapping the BAC core
 * @date 2026-08-27
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Thin adapter: transforms the nav2 plan into the robot frame, feeds the core
 * obstacle points (raw laser scan when scan_topic is set; lethal costmap cells
 * as fallback), and publishes the core's (v, w) output. All planning logic
 * lives in the framework-free core.
 *
 * Two nav2_core::Controller interfaces are implemented, selected by the
 * BAC_NAV2_API definition the build derives from the nav2_core version:
 *
 *  - 1: Nav2 1.3 (ROS 2 Jazzy) and 1.4 (Kilted). The parent node is an
 *    rclcpp_lifecycle::LifecycleNode, the plan arrives through setPlan(), and
 *    computeVelocityCommands() takes the pose, the velocity and the goal
 *    checker.
 *  - 2: Nav2 1.5 (ROS 2 Lyrical). The parent node is a nav2::LifecycleNode,
 *    the plan arrives through newPathReceived(), and computeVelocityCommands()
 *    also receives the controller server's transformed plan and the goal.
 *
 * The plan handling is the same under both: the plugin keeps the raw global
 * plan, transforms it into the base frame through TF on every tick, and
 * prunes it to `max_range` itself. Under interface 2 the plan the controller
 * server's path handler hands over is therefore not consumed: it is expressed
 * in the costmap's global frame and cut at the path handler's own
 * `prune_distance`, and taking it would change how much path the core sees by
 * a server parameter this package does not document or measure.
 */

#pragma once
#ifndef BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_
#define BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#ifndef BAC_NAV2_API
#error "BAC_NAV2_API must be defined: 1 for Nav2 <= 1.4 (Jazzy, Kilted), 2 for Nav2 >= 1.5 (Lyrical)"
#endif

#if BAC_NAV2_API >= 2
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#else
#include "tf2_ros/buffer.hpp"
#endif

namespace bac
{

#if BAC_NAV2_API >= 2
/// The parent node type the nav2_core::Controller interface names.
using Nav2Node = nav2::LifecycleNode;
/// The TF buffer type the nav2_core::Controller interface names.
using Nav2TfBuffer = nav2::TransformBuffer;
#else
using Nav2Node = rclcpp_lifecycle::LifecycleNode;
using Nav2TfBuffer = tf2_ros::Buffer;
#endif

class BacController : public nav2_core::Controller
{
public:
  BacController()           = default;
  ~BacController() override = default;

  void configure(const Nav2Node::WeakPtr &parent, std::string name,
                 std::shared_ptr<Nav2TfBuffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

#if BAC_NAV2_API >= 2
  /// Keeps the raw global plan; the transformation happens on every tick.
  void newPathReceived(const nav_msgs::msg::Path &raw_global_path) override;

  /// `transformed_global_plan` and `global_goal` are accepted and not used;
  /// see the file comment.
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker, const nav_msgs::msg::Path &transformed_global_plan,
      const geometry_msgs::msg::PoseStamped &global_goal) override;
#else
  void setPlan(const nav_msgs::msg::Path &path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                           const geometry_msgs::msg::Twist &velocity,
                                                           nav2_core::GoalChecker *goal_checker) override;
#endif

  /// Nav2 speed limit (Speed Filter, speed zones). Scales every velocity
  /// limit - forward, reverse, lateral and yaw rate - by one ratio, the way
  /// the MPPI and DWB controllers do, so a zone slows the robot in every
  /// direction it can move, turning included, and keeps the curvature it can
  /// drive. 0 (nav2_costmap_2d::NO_SPEED_LIMIT) or 100 % removes it. Called
  /// from the controller server's executor thread, concurrently with
  /// computeVelocityCommands().
  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

  /// Called by the controller server when a task exits (succeeded, cancelled
  /// or aborted): clears the core's temporal state - the AVOIDING latch,
  /// candidate hysteresis and the previous output - so the next task starts
  /// from the same state activate() leaves.
  void reset() override;

private:
  /// The one plan entry point behind setPlan() and newPathReceived()
  void acceptPlan(const nav_msgs::msg::Path &path);

  /// The one control tick behind both computeVelocityCommands() signatures
  geometry_msgs::msg::TwistStamped computeCommand(const geometry_msgs::msg::PoseStamped &pose,
                                                  const geometry_msgs::msg::Twist &velocity);

  /// Lethal costmap cells within max_range, in the robot frame
  std::vector<Point2D> collectObstaclePoints(const geometry_msgs::msg::PoseStamped &pose) const;

  /// Latest laser scan as robot-frame points (std::nullopt when no fresh scan)
  std::optional<std::vector<Point2D>> collectScanPoints();

  /// The current plan transformed into the robot frame
  /// Transforms the plan into the base frame and prunes it. `goal_heading`
  /// receives the orientation of the LAST plan pose, in the base frame, but
  /// only when that pose survived pruning - otherwise the far end of `path` is
  /// a waypoint rather than the goal, and its orientation is not a goal
  /// orientation.
  ///
  /// `path_yaw` (never null) receives one orientation per returned point, in
  /// the base frame, when plan_yaw_mode is "plan" - the pose orientations the
  /// plan carries, pruned on the pruner's own decision. Cleared otherwise.
  std::vector<Point2D> transformPlan(const geometry_msgs::msg::PoseStamped &pose,
                                     std::optional<float> *goal_heading,
                                     std::vector<float> *path_yaw) const;

  /// Publish the active obstacle source and selected-candidate diagnostics
  void publishDiagnostics(const Result &result, bool using_scan);

  /// Reject live changes to this plugin's parameters while it is active: they
  /// are read in configure(), and a change accepted there would be silently
  /// ignored. Inactive (between cleanup and configure) they are accepted, so
  /// the lifecycle reconfigure path keeps working.
  rcl_interfaces::msg::SetParametersResult
  onSetParameters(const std::vector<rclcpp::Parameter> &parameters) const;

  Nav2Node::WeakPtr                              parent_;
  std::string                                    name_;
  std::shared_ptr<Nav2TfBuffer>                  tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path                            plan_;

  // Direct laser input (the core's native point source). When scan_topic is
  // set, fresh scans feed the core; the costmap is a fallback.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr                  latest_scan_;
  std::mutex                                                   scan_mutex_;
  rclcpp::Clock::SharedPtr                                     clock_;
  std::string                                                  scan_topic_;
  float                                                        scan_timeout_ = 0.5f;  // [s]
  int                                                          scan_downsample_ = 1;
  int                                                          scan_min_points_ = 10;
  bool                                                         scan_inf_is_valid_ = true;
  std::string                                                  scan_state_ = "disabled";

  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
  float        diagnostics_publish_period_ = 1.0f;  // [s]
  rclcpp::Time last_diagnostics_time_{ 0, 0, RCL_ROS_TIME };

  rclcpp::Logger logger_{ rclcpp::get_logger("BacController") };
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::atomic<bool> active_{ false };

  BacCore core_;

  /// Configured velocity limits; the speed limit scales these.
  Limits base_limits_;
  /// Speed-limit ratio in (0, 1]; 1 = unlimited. Written by setSpeedLimit()
  /// on the executor thread, read by computeVelocityCommands().
  std::atomic<float> speed_ratio_{ 1.0f };

  /// plan_yaw_mode "plan": hand the plan's per-pose orientations to the core,
  /// so the plan owns the body orientation instead of the path tangent.
  /// Requires a holonomic model - configure() rejects it for the others.
  bool plan_yaw_ = false;
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_
