/**
 * @file ros_adapter_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/bac_controller.hpp"
#include "test_expect.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

namespace
{

using bac_test::expect;
using bac_test::failures;

std::string diagnosticValue(const diagnostic_msgs::msg::DiagnosticStatus &status,
                            const std::string &key)
{
  for (const auto &entry : status.values)
  {
    if (entry.key == key)
    {
      return entry.value;
    }
  }
  return {};
}

/// The plan entry point of the nav2_core::Controller interface this build
/// implements (see bac_controller.hpp): newPathReceived() from Nav2 1.5,
/// setPlan() before it.
void givePlan(bac::BacController &controller, const nav_msgs::msg::Path &path)
{
#if BAC_NAV2_API >= 2
  controller.newPathReceived(path);
#else
  controller.setPlan(path);
#endif
}

/// The control tick of that interface. Under Nav2 1.5 the controller server
/// also hands over the path handler's transformed plan and the goal; the
/// fixtures pass the plan itself and its last pose, which is what a path
/// handler produces for a plan already in the costmap frame.
geometry_msgs::msg::TwistStamped tick(bac::BacController &controller,
                                      const geometry_msgs::msg::PoseStamped &pose,
                                      const geometry_msgs::msg::Twist &velocity,
                                      const nav_msgs::msg::Path &plan)
{
#if BAC_NAV2_API >= 2
  geometry_msgs::msg::PoseStamped goal;
  if (!plan.poses.empty())
  {
    goal = plan.poses.back();
    goal.header.frame_id = plan.header.frame_id;
  }
  return controller.computeVelocityCommands(pose, velocity, nullptr, plan, goal);
#else
  (void)plan;
  return controller.computeVelocityCommands(pose, velocity, nullptr);
#endif
}

void spinFor(rclcpp::executors::SingleThreadedExecutor &executor,
             std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

/// A costmap whose configure step returns only once its executor thread is
/// spinning. Costmap2DROS stops that thread on cleanup through Nav2's
/// NodeThread, which calls Executor::cancel() and joins; cancel() only ends a
/// spin already running, so cleaning up before the thread reaches spin() (a
/// loaded machine, and these fixtures clean up within milliseconds) joins a
/// thread that never returns.
class SpinningCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  using nav2_costmap_2d::Costmap2DROS::Costmap2DROS;

  void configureAndWaitForSpin()
  {
    configure();
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (!(executor_ && executor_->is_spinning()) &&
           std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(1ms);
    }
  }
};

nav_msgs::msg::Path straightPlan(const std::string &frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = 0.2 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path rearPlan(const std::string &frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = -0.2 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

/// A plan drawn straight to the robot's LEFT whose every pose keeps the
/// robot's own orientation (yaw 0): under plan_yaw_mode "plan" the vehicle
/// must crab along it rather than turn onto the tangent.
nav_msgs::msg::Path sidePlanHoldingYaw(const std::string &frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.y = 0.2 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;  // yaw 0 everywhere, the goal pose included
    path.poses.push_back(pose);
  }
  return path;
}


void testControllerAdapter()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.scan_topic", "/scan"),
    rclcpp::Parameter("FollowPath.scan_timeout", 0.5),
    rclcpp::Parameter("FollowPath.scan_min_points", 3),
    rclcpp::Parameter("FollowPath.diagnostics_publish_period", 0.001),
    rclcpp::Parameter("FollowPath.limits.v_min", 0.0)
  });
  auto parent = std::make_shared<bac::Nav2Node>("bac_adapter_test", options);
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.1)
  });
  auto costmap = std::make_shared<SpinningCostmap>(costmap_options);
  costmap->configureAndWaitForSpin();
  auto tf = std::make_shared<bac::Nav2TfBuffer>(parent->get_clock());

  bac::BacController controller;
  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();

  auto driver = std::make_shared<rclcpp::Node>("bac_adapter_test_driver");
  auto scan_pub = driver->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
  diagnostic_msgs::msg::DiagnosticStatus last_diagnostic;
  bool diagnostic_received = false;
  auto diagnostic_sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        if (!message->status.empty())
        {
          last_diagnostic = message->status.front();
          diagnostic_received = true;
        }
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent->get_node_base_interface());
  executor.add_node(driver);
  spinFor(executor, 100ms);

  const std::string base_frame = costmap->getBaseFrameID();
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  nav_msgs::msg::Path plan = straightPlan(base_frame);
  givePlan(controller, plan);

  // With no scan received, the configured raw source must visibly fall back.
  tick(controller, robot_pose, velocity, plan);
  spinFor(executor, 50ms);
  expect(diagnostic_received, "controller publishes diagnostics while active");
  expect(last_diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::WARN,
         "configured raw scan fallback is WARN-level");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "costmap_fallback",
         "diagnostics expose costmap fallback");
  expect(diagnosticValue(last_diagnostic, "scan_state") == "not_received",
         "diagnostics expose the fallback reason");

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = base_frame;
  scan.angle_min = -0.5f;
  scan.angle_increment = 0.1f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.assign(11U, std::numeric_limits<float>::infinity());
  scan.header.stamp = driver->now();
  scan_pub->publish(scan);
  spinFor(executor, 100ms);

  diagnostic_received = false;
  const geometry_msgs::msg::TwistStamped command = tick(controller, robot_pose, velocity, plan);
  spinFor(executor, 50ms);
  expect(std::isfinite(command.twist.linear.x) && std::isfinite(command.twist.angular.z),
         "fresh clear scan produces a finite command");
  expect(diagnostic_received, "fresh scan decision publishes diagnostics");
  expect(last_diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::OK,
         "fresh scan is OK-level");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "raw_scan",
         "diagnostics expose raw scan use");

  // Half of the model-selection check. The DEFAULT configuration must still be
  // differential drive, which turns on the spot for a plan lying entirely
  // behind the vehicle. testAckermannAdapterConfiguration asserts the other
  // half, so a motion_model.type that silently resolved to the wrong policy
  // fails one of the two.
  const nav_msgs::msg::Path rear_plan = rearPlan(base_frame);
  givePlan(controller, rear_plan);
  const geometry_msgs::msg::TwistStamped diff_rear_command =
      tick(controller, robot_pose, velocity, rear_plan);
  expect(std::fabs(diff_rear_command.twist.linear.x) <= 1e-4 &&
             std::fabs(diff_rear_command.twist.angular.z) > 1e-3,
         "the default Nav2 configuration is differential drive and turns on the spot");
  givePlan(controller, plan);

  // Nav2 speed limiting must cap the output selected from an otherwise clear scan.
  controller.setSpeedLimit(0.05, false);
  const geometry_msgs::msg::TwistStamped limited = tick(controller, robot_pose, velocity, plan);
  expect(limited.twist.linear.x <= 0.05001,
         "absolute Nav2 speed limit caps the selected command");

  // A once-valid scan must become a visible fallback after its timeout.
  spinFor(executor, 550ms);
  diagnostic_received = false;
  tick(controller, robot_pose, velocity, plan);
  spinFor(executor, 20ms);
  expect(diagnostic_received, "stale scan decision publishes diagnostics");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "costmap_fallback" &&
             diagnosticValue(last_diagnostic, "scan_state") == "stale",
         "stale scan falls back with an explicit reason");

  // A fresh message with too few valid rays is a sensor fault, not empty space.
  scan.header.stamp = driver->now();
  scan.ranges.assign(11U, std::numeric_limits<float>::quiet_NaN());
  scan_pub->publish(scan);
  spinFor(executor, 50ms);
  diagnostic_received = false;
  tick(controller, robot_pose, velocity, plan);
  spinFor(executor, 20ms);
  expect(diagnostic_received, "invalid scan decision publishes diagnostics");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "costmap_fallback" &&
             diagnosticValue(last_diagnostic, "scan_state") == "insufficient_valid_rays",
         "invalid scan falls back with an explicit reason");

  // Parameters are read at configure. While active, a change would be
  // accepted and ignored, so it is refused; parameters outside the plugin's
  // prefix are none of its business.
  parent->declare_parameter("unrelated_value", 1.0);
  const auto refused = parent->set_parameter(rclcpp::Parameter("FollowPath.limits.v_max", 0.1));
  expect(!refused.successful, "an active controller refuses a change to its own parameter");
  expect(refused.reason.find("FollowPath.limits.v_max") != std::string::npos,
         "the refusal names the parameter");
  expect(parent->set_parameter(rclcpp::Parameter("unrelated_value", 2.0)).successful,
         "a parameter outside the plugin's prefix is left alone");

  nav_msgs::msg::Path unavailable_plan = straightPlan("unavailable_frame");
  givePlan(controller, unavailable_plan);
  bool threw_tf_error = false;
  try
  {
    tick(controller, robot_pose, velocity, unavailable_plan);
  }
  catch (const nav2_core::ControllerTFError &)
  {
    threw_tf_error = true;
  }
  expect(threw_tf_error, "unavailable plan transform raises ControllerTFError");

  controller.deactivate();
  expect(parent->set_parameter(rclcpp::Parameter("FollowPath.limits.v_max", 0.3)).successful,
         "a deactivated controller accepts the change the next configure will read");
  controller.cleanup();
  costmap->cleanup();
  executor.remove_node(parent->get_node_base_interface());
  executor.remove_node(driver);
  (void)diagnostic_sub;
}

/// The Ackermann parameters must reach BacCore through the plugin boundary.
/// Runs on its own lifecycle and costmap nodes so it cannot disturb the
/// default-configuration coverage above.
void testAckermannAdapterConfiguration()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.scan_topic", "/scan"),
    rclcpp::Parameter("FollowPath.scan_timeout", 0.5),
    rclcpp::Parameter("FollowPath.scan_min_points", 3),
    rclcpp::Parameter("FollowPath.limits.v_min", 0.0),
    rclcpp::Parameter("FollowPath.motion_model.type", "ackermann"),
    rclcpp::Parameter("FollowPath.turn_radius_min", 0.8)
  });
  auto parent = std::make_shared<bac::Nav2Node>("bac_ackermann_adapter_test", options);
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.1)
  });
  // A distinct node name keeps this costmap's parameters and topics separate
  // from the default-configuration test's.
  costmap_options.arguments({ "--ros-args", "-r", "__node:=ackermann_costmap" });
  auto costmap = std::make_shared<SpinningCostmap>(costmap_options);
  costmap->configureAndWaitForSpin();
  auto tf = std::make_shared<bac::Nav2TfBuffer>(parent->get_clock());

  bac::BacController controller;
  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();

  const std::string base_frame = "base_link";
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;

  // The other half of the model-selection check: where differential drive
  // turns on the spot, a forward-only Ackermann vehicle must brake instead.
  const nav_msgs::msg::Path rear_plan = rearPlan(base_frame);
  givePlan(controller, rear_plan);
  const geometry_msgs::msg::TwistStamped rear_command =
      tick(controller, robot_pose, velocity, rear_plan);
  expect(std::fabs(rear_command.twist.linear.x) <= 1e-4 &&
             std::fabs(rear_command.twist.angular.z) <= 1e-4,
         "forward-only Ackermann brakes for a rear plan instead of yawing");

  // Deliberately NOT asserted here: the minimum turning radius. In this
  // fixture the vehicle is at standstill and the selected arc is always
  // straight, so a radius assertion would be unreachable code that reads like
  // coverage it does not provide. The turning-radius bound - including the
  // binding case, the reverse case and the reachability-clamp interaction - is
  // covered by bac_ackermann_motion_model_unit and bac_ackermann_scenarios.

  controller.deactivate();
  controller.cleanup();
  costmap->cleanup();
}

void testPlanYawAdapterConfiguration()
{
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.1)
  });
  costmap_options.arguments({ "--ros-args", "-r", "__node:=plan_yaw_costmap" });
  auto costmap = std::make_shared<SpinningCostmap>(costmap_options);
  costmap->configureAndWaitForSpin();

  // Handing the orientation to the plan is meaningless for a model that
  // steers with yaw, and silently ignoring the request would leave the user
  // believing the plan governs. configure() must refuse, not comply halfway.
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.plan_yaw_mode", "plan"),
    });
    auto parent = std::make_shared<bac::Nav2Node>("bac_plan_yaw_reject_test", options);
    auto tf = std::make_shared<bac::Nav2TfBuffer>(parent->get_clock());
    bac::BacController controller;
    bool threw = false;
    try
    {
      controller.configure(parent, "FollowPath", tf, costmap);
    }
    catch (const std::invalid_argument &)
    {
      threw = true;
    }
    expect(threw, "plan_yaw_mode 'plan' under differential drive fails configure, not silently");
  }

  // An unknown mode is a typo, and a typo that silently means "off" would
  // read as the feature being broken rather than misspelled.
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.motion_model.type", "omni"),
      rclcpp::Parameter("FollowPath.limits.vy_max", 0.3),
      rclcpp::Parameter("FollowPath.plan_yaw_mode", "sideways"),
    });
    auto parent = std::make_shared<bac::Nav2Node>("bac_plan_yaw_badmode_test", options);
    auto tf = std::make_shared<bac::Nav2TfBuffer>(parent->get_clock());
    bac::BacController controller;
    bool threw = false;
    try
    {
      controller.configure(parent, "FollowPath", tf, costmap);
    }
    catch (const std::invalid_argument &)
    {
      threw = true;
    }
    expect(threw, "an unknown plan_yaw_mode fails configure instead of meaning 'off'");
  }

  // End to end through the adapter: the same sideways plan, once with the
  // plan owning the orientation and once without. Asserted as the DIFFERENCE:
  // "plan" crabs (lateral motion, yaw exactly zero - every pose asks for the
  // orientation the body already has), "off" yaws onto the tangent.
  auto run_side_plan = [&](const char *node_name, const char *mode) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.motion_model.type", "omni"),
      rclcpp::Parameter("FollowPath.limits.vy_max", 0.3),
      rclcpp::Parameter("FollowPath.limits.v_min", 0.0),
      rclcpp::Parameter("FollowPath.plan_yaw_mode", mode),
    });
    auto parent = std::make_shared<bac::Nav2Node>(node_name, options);
    auto tf = std::make_shared<bac::Nav2TfBuffer>(parent->get_clock());
    bac::BacController controller;
    controller.configure(parent, "FollowPath", tf, costmap);
    controller.activate();
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "base_link";
    robot_pose.pose.orientation.w = 1.0;
    geometry_msgs::msg::Twist velocity;
    const nav_msgs::msg::Path side_plan = sidePlanHoldingYaw("base_link");
    givePlan(controller, side_plan);
    const geometry_msgs::msg::TwistStamped command =
        tick(controller, robot_pose, velocity, side_plan);
    controller.deactivate();
    controller.cleanup();
    return command;
  };
  const geometry_msgs::msg::TwistStamped crab = run_side_plan("bac_plan_yaw_test", "plan");
  const geometry_msgs::msg::TwistStamped turn = run_side_plan("bac_plan_yaw_off_test", "off");
  expect(crab.twist.angular.z == 0.0,
         "under 'plan' a sideways plan holding yaw 0 commands no rotation at all");
  expect(crab.twist.linear.y > 0.0,
         "under 'plan' the journey to a plan abeam goes into linear.y (" +
             std::to_string(crab.twist.linear.y) + " m/s)");
  expect(std::fabs(turn.twist.angular.z) > std::fabs(crab.twist.angular.z),
         "under 'off' the same plan is turned onto instead (" +
             std::to_string(turn.twist.angular.z) + " rad/s)");

  costmap->cleanup();
}

/// The lifecycle reconfigure path (deactivate, cleanup, change, configure,
/// activate) must apply the new value, reset() must start the next task
/// clean, and the Nav2 speed limit must bound reverse speed, not only forward.
void testReconfigureAndSpeedLimit()
{
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.1)
  });
  costmap_options.arguments({ "--ros-args", "-r", "__node:=reconfigure_costmap" });
  auto costmap = std::make_shared<SpinningCostmap>(costmap_options);
  costmap->configureAndWaitForSpin();

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.motion_model.type", "omni"),
    rclcpp::Parameter("FollowPath.limits.vy_max", 0.3),
    rclcpp::Parameter("FollowPath.limits.v_min", 0.0),
    rclcpp::Parameter("FollowPath.plan_yaw_mode", "plan"),
  });
  auto parent = std::make_shared<bac::Nav2Node>("bac_reconfigure_test", options);
  auto tf = std::make_shared<bac::Nav2TfBuffer>(parent->get_clock());
  bac::BacController controller;
  // The controller server hands over the pose in the costmap's global frame.
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  const nav_msgs::msg::Path side_plan = sidePlanHoldingYaw("base_link");

  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();
  givePlan(controller, side_plan);
  const geometry_msgs::msg::TwistStamped crab = tick(controller, robot_pose, velocity, side_plan);
  expect(crab.twist.angular.z == 0.0 && crab.twist.linear.y > 0.0,
         "configured with plan_yaw_mode 'plan', the side plan is crabbed");
  expect(crab.header.frame_id == "base_link",
         "the command is expressed in the base frame, got '" + crab.header.frame_id + "'");

  controller.deactivate();
  controller.cleanup();
  expect(parent->set_parameter(rclcpp::Parameter("FollowPath.plan_yaw_mode", "off")).successful,
         "an unconfigured controller's parameters can be changed");
  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();
  givePlan(controller, side_plan);
  const geometry_msgs::msg::TwistStamped turn = tick(controller, robot_pose, velocity, side_plan);
  expect(std::fabs(turn.twist.angular.z) > 0.0,
         "after reconfiguring to 'off' the same plan is turned onto, not crabbed (" +
             std::to_string(turn.twist.angular.z) + " rad/s)");

  // Goal exit: the controller server calls reset(); the next task starts clean.
  controller.reset();
  const geometry_msgs::msg::TwistStamped after_reset =
      tick(controller, robot_pose, velocity, side_plan);
  expect(after_reset.twist.angular.z == turn.twist.angular.z &&
             after_reset.twist.linear.x == turn.twist.linear.x,
         "after reset() the controller answers the first tick of a task as it did after activate()");

  controller.deactivate();
  controller.cleanup();

  // The Nav2 speed limit bounds the speed in every direction. Capping
  // limits.v_max alone left reverse at limits.v_min: a differential-drive (or
  // Ackermann) base configured for reverse backed up at its full reverse speed
  // inside a speed zone. The fixture makes reverse the only way out: at a
  // standstill with an obstacle cell inside the front stopping margin, the
  // core offers only an escape away from it. An absolute limit of 0.01 m/s is
  // 2.5 % of limits.v_max (0.4), so every translational bound scales to 2.5 %
  // and the escape may not exceed it (stopping instead is within the limit);
  // with limits.v_min unscaled the escape backed away at 0.1 m/s.
  // (The holonomic model's admissibility test already bounds its speed norm
  // by limits.v_max; lateral speed is scaled with the rest for the same
  // reason.)
  {
    unsigned int mx = 0, my = 0;
    const bool in_map = costmap->getCostmap()->worldToMap(0.55, 0.05, mx, my);
    expect(in_map, "the speed-limit fixture's obstacle cell lies inside the costmap");
    costmap->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

    rclcpp::NodeOptions reverse_options;
    reverse_options.parameter_overrides({
      rclcpp::Parameter("FollowPath.limits.v_min", -0.3),
    });
    auto reverse_parent =
        std::make_shared<bac::Nav2Node>("bac_speed_limit_test", reverse_options);
    auto reverse_tf = std::make_shared<bac::Nav2TfBuffer>(reverse_parent->get_clock());
    bac::BacController limited;
    limited.configure(reverse_parent, "FollowPath", reverse_tf, costmap);
    limited.activate();
    const nav_msgs::msg::Path ahead = straightPlan("base_link");
    givePlan(limited, ahead);
    const geometry_msgs::msg::Twist standstill;

    const geometry_msgs::msg::TwistStamped free = tick(limited, robot_pose, standstill, ahead);
    expect(free.twist.linear.x < -0.02,
           "without a speed limit the fixture escapes in reverse (" +
               std::to_string(free.twist.linear.x) + " m/s)");

    limited.setSpeedLimit(0.01, false);
    limited.reset();
    const geometry_msgs::msg::TwistStamped slow = tick(limited, robot_pose, standstill, ahead);
    expect(std::fabs(slow.twist.linear.x) <= 0.01 + 1e-6,
           "a 0.01 m/s Nav2 speed limit holds in reverse (" +
               std::to_string(slow.twist.linear.x) + " m/s)");

    limited.setSpeedLimit(5.0, true);
    limited.reset();
    const geometry_msgs::msg::TwistStamped percent = tick(limited, robot_pose, standstill, ahead);
    expect(std::fabs(percent.twist.linear.x) <= 0.02 + 1e-6,
           "a 5 % Nav2 speed limit (0.02 m/s) holds in reverse (" +
               std::to_string(percent.twist.linear.x) + " m/s)");

    limited.setSpeedLimit(0.0, false);  // nav2_costmap_2d::NO_SPEED_LIMIT
    limited.reset();
    const geometry_msgs::msg::TwistStamped restored = tick(limited, robot_pose, standstill, ahead);
    expect(restored.twist.linear.x == free.twist.linear.x &&
               restored.twist.angular.z == free.twist.angular.z,
           "removing the speed limit restores the configured bounds exactly");

    limited.deactivate();
    limited.cleanup();
    costmap->getCostmap()->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  }
  costmap->cleanup();
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testControllerAdapter();
  testAckermannAdapterConfiguration();
  testPlanYawAdapterConfiguration();
  testReconfigureAndSpeedLimit();
  rclcpp::shutdown();

  if (failures != 0)
  {
    std::cerr << failures << " ROS adapter check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC ROS adapter checks passed\n";
  return 0;
}
