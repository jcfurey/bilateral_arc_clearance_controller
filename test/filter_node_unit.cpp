/**
 * @file filter_node_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * bac_filter_node in-process, through its component class. What is checked is
 * the ROS surface, not the planner: the tick period and its clock, both
 * cmd_vel message types, and that parameter changes the node would ignore
 * are refused.
 */

#include "bac_filter_node.hpp"
#include "test_expect.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

namespace
{

using bac_test::expect;
using bac_test::failures;

/// A filter node in its own namespace, so the checks cannot hear each other.
std::shared_ptr<bac::BacFilterNode>
makeFilter(const std::string &ns, std::vector<rclcpp::Parameter> overrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(std::move(overrides));
  options.arguments({ "--ros-args", "-r", "__ns:=/" + ns });
  return std::make_shared<bac::BacFilterNode>(options);
}

/// Publishes what the filter needs to pass a command through: a clear scan,
/// zero odometry and the command itself, each call.
class Driver
{
public:
  Driver(const rclcpp::Node::SharedPtr &node, const std::string &ns, bool stamped)
    : node_(node), stamped_(stamped)
  {
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("/" + ns + "/scan",
                                                                      rclcpp::SensorDataQoS());
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/" + ns + "/odom", 10);
    if (stamped_)
    {
      cmd_stamped_pub_ =
          node_->create_publisher<geometry_msgs::msg::TwistStamped>("/" + ns + "/cmd_vel_in", 10);
    }
    else
    {
      cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/" + ns + "/cmd_vel_in", 10);
    }
  }

  void publish(double v)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.angle_min       = -1.5f;
    scan.angle_increment = 0.1f;
    scan.range_min       = 0.1f;
    scan.range_max       = 10.0f;
    scan.ranges.assign(31U, std::numeric_limits<float>::infinity());
    scan_pub_->publish(scan);
    odom_pub_->publish(nav_msgs::msg::Odometry());
    if (stamped_)
    {
      geometry_msgs::msg::TwistStamped command;
      command.header.frame_id = "base_link";
      command.twist.linear.x  = v;
      cmd_stamped_pub_->publish(command);
    }
    else
    {
      geometry_msgs::msg::Twist command;
      command.linear.x = v;
      cmd_pub_->publish(command);
    }
  }

private:
  rclcpp::Node::SharedPtr                                        node_;
  bool                                                           stamped_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr      scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
};

void spinFor(rclcpp::executors::SingleThreadedExecutor &executor, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

/// The node's defaults: the tick period is control_period and the cmd_vel
/// type follows the distribution's Nav2.
void testDefaults()
{
  auto filter = makeFilter("defaults", {});
  expect(filter->controlPeriod() == static_cast<double>(bac::Params().control_period),
         "the tick period defaults to the core's control_period");
  expect(filter->stampedCmdVel() == bac::defaultStampedCmdVel(),
         "enable_stamped_cmd_vel defaults to the distribution's Nav2 cmd_vel type");
}

/// The tick runs every control_period, not at a fixed 20 Hz. At 0.2 s a
/// second of spinning allows about five ticks; the fixed 50 ms timer this
/// replaced gave about twenty. Only an upper bound can be asserted robustly
/// against a loaded machine, and it is what separates the two.
void testTickPeriodFollowsControlPeriod()
{
  auto filter = makeFilter("period", { rclcpp::Parameter("control_period", 0.2) });
  auto probe  = std::make_shared<rclcpp::Node>("period_probe");
  int  ticks  = 0;
  auto sub    = probe->create_subscription<std_msgs::msg::Int8>(
      "/period/avoid_status", 10, [&](std_msgs::msg::Int8::ConstSharedPtr) { ++ticks; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter);
  executor.add_node(probe);
  spinFor(executor, 1000ms);
  expect(ticks >= 2 && ticks <= 8, "control_period 0.2 s gives about 5 ticks per second, got " +
                                       std::to_string(ticks));
}

/// Under use_sim_time the tick follows /clock: without it, simulated time
/// stands still and the node must not tick; once /clock advances, it does.
/// A wall-clock timer ticks in both phases.
void testTickFollowsSimulatedTime()
{
  auto filter = makeFilter("simtime", { rclcpp::Parameter("use_sim_time", true) });
  auto probe  = std::make_shared<rclcpp::Node>("simtime_probe");
  int  ticks  = 0;
  auto sub    = probe->create_subscription<std_msgs::msg::Int8>(
      "/simtime/avoid_status", 10, [&](std_msgs::msg::Int8::ConstSharedPtr) { ++ticks; });
  auto clock_pub = probe->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter);
  executor.add_node(probe);

  spinFor(executor, 400ms);
  expect(ticks == 0, "with use_sim_time and no /clock the filter does not tick (got " +
                         std::to_string(ticks) + ")");

  rosgraph_msgs::msg::Clock clock;
  for (int i = 1; i <= 40; ++i)
  {
    clock.clock.sec     = 100 + i / 20;
    clock.clock.nanosec = static_cast<uint32_t>((i % 20) * 50000000);
    clock_pub->publish(clock);
    spinFor(executor, 10ms);
  }
  expect(ticks > 0, "once /clock advances the filter ticks on simulated time");
}

/// Both cmd_vel message types pass a command through, and the stamped
/// output keeps the input's frame.
void testCommandTypes()
{
  for (bool stamped : { false, true })
  {
    const std::string ns = stamped ? "stamped" : "unstamped";
    auto filter = makeFilter(ns, { rclcpp::Parameter("enable_stamped_cmd_vel", stamped) });
    auto probe  = std::make_shared<rclcpp::Node>(ns + "_probe");
    Driver driver(probe, ns, stamped);
    double      out_v = 0.0;
    std::string out_frame;
    int         outputs = 0;
    rclcpp::SubscriptionBase::SharedPtr sub;
    if (stamped)
    {
      sub = probe->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/" + ns + "/cmd_vel_out", 10,
          [&](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
            out_v     = msg->twist.linear.x;
            out_frame = msg->header.frame_id;
            ++outputs;
          });
    }
    else
    {
      sub = probe->create_subscription<geometry_msgs::msg::Twist>(
          "/" + ns + "/cmd_vel_out", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            out_v = msg->linear.x;
            ++outputs;
          });
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(filter);
    executor.add_node(probe);
    for (int i = 0; i < 40; ++i)
    {
      driver.publish(0.2);
      spinFor(executor, 20ms);
    }
    expect(filter->stampedCmdVel() == stamped, ns + ": the node uses the configured type");
    expect(outputs > 0, ns + ": the filter publishes on cmd_vel_out");
    expect(out_v > 0.0, ns + ": a clear scan passes the forward command through (" +
                            std::to_string(out_v) + " m/s)");
    if (stamped)
    {
      expect(out_frame == "base_link", "stamped: the output keeps the input frame, got '" +
                                           out_frame + "'");
    }
  }
}

/// A parameter the node read at startup cannot be changed silently.
void testParameterChangesAreRefused()
{
  auto filter = makeFilter("params", {});
  const auto refused = filter->set_parameter(rclcpp::Parameter("limits.v_max", 0.1));
  expect(!refused.successful, "changing limits.v_max at runtime is refused");
  expect(refused.reason.find("limits.v_max") != std::string::npos,
         "the refusal names the parameter");
  const auto adapter = filter->set_parameter(rclcpp::Parameter("cmd_timeout", 2.0));
  expect(!adapter.successful, "changing an adapter parameter at runtime is refused");
  expect(filter->get_parameter("limits.v_max").as_double() != 0.1,
         "the refused value is not stored");
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testDefaults();
  testTickPeriodFollowsControlPeriod();
  testTickFollowsSimulatedTime();
  testCommandTypes();
  testParameterChangesAreRefused();
  rclcpp::shutdown();

  if (failures != 0)
  {
    std::cerr << failures << " filter node check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC filter node checks passed\n";
  return 0;
}
