/**
 * @file bac_filter_node.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief ROS 2 evaluation node for bac_core: cmd_vel filter driven by a laser scan
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * See bac_filter_node.hpp for the topics. Built as the composable node
 * bac::BacFilterNode; the bac_filter_node executable runs it standalone.
 */

#include "bac_filter_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "bac_ros_parameters.hpp"
#include "bilateral_arc_clearance_controller/adapter_utils.hpp"
#include "rclcpp/version.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace bac
{

bool
defaultStampedCmdVel()
{
  // Nav2 switched its cmd_vel publishers to TwistStamped by default in Kilted,
  // whose rclcpp is 29.x (Jazzy 28.x, Lyrical 32.x).
#if RCLCPP_VERSION_MAJOR >= 29
  return true;
#else
  return false;
#endif
}

BacFilterNode::BacFilterNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("bac_filter", options)
{
  Params params = ros_parameters::declareCoreParameters(*this, "", &parameter_names_);
  if (!(params.control_period > 0.0f) || !std::isfinite(params.control_period))
  {
    throw std::invalid_argument("bac: control_period must be positive");
  }
  core_.setParams(params);
  base_v_max_          = params.limits.v_max;
  max_range_           = params.max_range;
  control_period_      = static_cast<double>(params.control_period);
  virtual_path_length_ = declareFloat("virtual_path_length", 3.0f);

  // 2D pose of the laser in the robot frame
  sensor_x_   = declareFloat("sensor.x", 0.0f);
  sensor_y_   = declareFloat("sensor.y", 0.0f);
  sensor_yaw_ = declareFloat("sensor.yaw", 0.0f);

  scan_timeout_ = declareFloat("scan_timeout", 0.5f);
  cmd_timeout_  = declareFloat("cmd_timeout", 0.5f);
  odom_timeout_ = declareFloat("odom_timeout", 0.5f);
  parameter_names_.push_back("scan_min_points");
  scan_min_points_ = static_cast<int>(
      std::max<int64_t>(0, declare_parameter<int64_t>("scan_min_points", 10)));
  parameter_names_.push_back("scan_inf_is_valid");
  scan_inf_is_valid_ = declare_parameter<bool>("scan_inf_is_valid", true);
  parameter_names_.push_back("enable_stamped_cmd_vel");
  stamped_ = declare_parameter<bool>("enable_stamped_cmd_vel", defaultStampedCmdVel());

  status_pub_ = create_publisher<std_msgs::msg::Int8>("avoid_status", 10);
  if (stamped_)
  {
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_out", 10);
    cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel_in", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          onCommand(msg->twist, msg->header.frame_id);
        });
  }
  else
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          onCommand(*msg, std::string());
        });
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_ = Twist2D(static_cast<float>(msg->twist.twist.linear.x),
                           static_cast<float>(msg->twist.twist.angular.z),
                           static_cast<float>(msg->twist.twist.linear.y));
        last_odom_time_ = now();
      });

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        ScanProjection projected =
            projectScan(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min,
                        msg->range_max, max_range_, 1, scan_inf_is_valid_,
                        sensor_x_, sensor_y_, sensor_yaw_);
        std::lock_guard<std::mutex> lock(mutex_);
        points_         = std::move(projected.points);
        valid_rays_     = static_cast<int>(projected.valid_ray_count);
        last_scan_time_ = now();
      });

  // The node clock, not the wall clock: under use_sim_time the tick follows
  // simulated time, and the period is the control_period the core's
  // acceleration and yaw-rate limits assume.
  timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(control_period_),
                                [this]() { tick(); });

  parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        return onSetParameters(parameters);
      });

  RCLCPP_INFO(get_logger(), "bac_filter running at %.1f Hz (%s cmd_vel)",
              1.0 / control_period_, stamped_ ? "TwistStamped" : "Twist");
}

float
BacFilterNode::declareFloat(const std::string &name, float default_value)
{
  parameter_names_.push_back(name);
  return static_cast<float>(declare_parameter<double>(name, static_cast<double>(default_value)));
}

void
BacFilterNode::onCommand(const geometry_msgs::msg::Twist &twist, const std::string &frame_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  command_ = Twist2D(static_cast<float>(twist.linear.x), static_cast<float>(twist.angular.z),
                     static_cast<float>(twist.linear.y));
  command_frame_ = frame_id;
  last_cmd_time_ = now();
}

rcl_interfaces::msg::SetParametersResult
BacFilterNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) const
{
  // Every parameter here is read once, at construction. Accepting a change
  // would report success for a value the node never uses.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &parameter : parameters)
  {
    if (std::find(parameter_names_.begin(), parameter_names_.end(), parameter.get_name()) !=
        parameter_names_.end())
    {
      result.successful = false;
      result.reason = "bac_filter: " + parameter.get_name() +
                      " is read at startup; restart the node to change it";
      break;
    }
  }
  return result;
}

void
BacFilterNode::tick()
{
  Twist2D              command, current;
  std::vector<Point2D> points;
  bool                 scan_fresh, cmd_fresh, odom_fresh;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp = now();
    command = command_;
    current = current_;
    points  = points_;
    // A scan that is fresh by timestamp but carries almost no valid
    // measurements (hits or explicit no-returns) is a sensor fault, not
    // an empty world.
    scan_fresh = last_scan_time_.nanoseconds() > 0 &&
                 (stamp - last_scan_time_).seconds() < static_cast<double>(scan_timeout_) &&
                 valid_rays_ >= scan_min_points_;
    cmd_fresh  = last_cmd_time_.nanoseconds() > 0 &&
                 (stamp - last_cmd_time_).seconds() < static_cast<double>(cmd_timeout_);
    odom_fresh = last_odom_time_.nanoseconds() > 0 &&
                 (stamp - last_odom_time_).seconds() < static_cast<double>(odom_timeout_);
  }

  if (!scan_fresh || !odom_fresh)
  {
    // No valid observation, or no velocity feedback to compute braking
    // distances from: stop rather than act on stale state.
    core_.forceStop();
    publish(Twist2D(), Status::STOP);
    return;
  }
  if (!cmd_fresh)
  {
    // Upstream went silent: do not keep executing its last command.
    publish(Twist2D(), core_.status());
    return;
  }

  // The upper (v, w) command becomes a virtual local path: the commanded
  // arc extended over virtual_path_length. The core then plans towards it
  // (and never faster than the commanded speed).
  std::vector<Point2D> path;
  float cv = command.v, cw = command.w;
  if (std::fabs(cv) > 0.02f || std::fabs(cw) > 0.02f)
  {
    float v_dir = (cv >= 0.0f) ? 1.0f : -1.0f;
    float v_eff = std::max(std::fabs(cv), 0.15f) * v_dir;  // pure turns get a short arc
    int   n     = 30;
    path.reserve(static_cast<std::size_t>(n));
    for (int i = 1; i <= n; i++)
    {
      float s = virtual_path_length_ * static_cast<float>(i) / static_cast<float>(n) * v_dir;
      if (std::fabs(cw) < 1e-3f)
      {
        path.emplace_back(s, 0.0f);
      }
      else
      {
        float radius = v_eff / cw;
        float theta  = s / radius;
        path.emplace_back(radius * std::sin(theta), radius * (1.0f - std::cos(theta)));
      }
    }
  }

  Params params = core_.params();
  float  v_cap  = std::min(base_v_max_, std::fabs(cv));
  if (params.limits.v_max != v_cap)
  {
    params.limits.v_max = v_cap;
    core_.setParams(params);
  }

  Result result = core_.process(points, path, current);
  // Arbitration: transparent while CLEAR - except the angular rate
  // limit, which is part of the reachability contract and therefore
  // applies to the passed-through command as well.
  Twist2D applied = result.output;
  if (result.status == Status::CLEAR)
  {
    applied = core_.limitReachableCommand(current, command);
  }
  publish(applied, result.status);
}

void
BacFilterNode::publish(const Twist2D &command, Status status)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = command.v;
  twist.linear.y  = command.vy;  // zero for every non-holonomic model
  twist.angular.z = command.w;
  if (stamped_)
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    {
      // The output is the same body velocity the input was expressed in.
      std::lock_guard<std::mutex> lock(mutex_);
      message.header.frame_id = command_frame_;
    }
    message.twist = twist;
    cmd_stamped_pub_->publish(message);
  }
  else
  {
    cmd_pub_->publish(twist);
  }
  std_msgs::msg::Int8 status_message;
  status_message.data = static_cast<int8_t>(status);
  status_pub_->publish(status_message);
}

}  // namespace bac

RCLCPP_COMPONENTS_REGISTER_NODE(bac::BacFilterNode)
