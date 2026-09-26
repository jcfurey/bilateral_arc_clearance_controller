/**
 * @file bac_controller.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Nav2 controller plugin wrapping the BAC core
 * @date 2026-08-27
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/bac_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "bac_ros_parameters.hpp"
#include "bilateral_arc_clearance_controller/adapter_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace bac
{

void
BacController::configure(const Nav2Node::WeakPtr &parent, std::string name,
                         std::shared_ptr<Nav2TfBuffer> tf,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_      = parent;
  name_        = name;
  tf_          = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node)
  {
    throw nav2_core::ControllerException("bac: unable to lock the controller server node");
  }
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configuring controller: %s of type bac::BacController", name.c_str());
  // configure() may run again after cleanup(); state set by a previous
  // configuration must not survive into this one.
  plan_yaw_ = false;
  speed_ratio_.store(1.0f);

  auto declare_float = [&](const std::string &param_name, float default_value) {
    const std::string full_name = name + "." + param_name;
    if (!node->has_parameter(full_name))
    {
      node->declare_parameter<double>(full_name, static_cast<double>(default_value));
    }
    return static_cast<float>(node->get_parameter(full_name).as_double());
  };
  Params params = ros_parameters::declareCoreParameters(*node, name + ".");
  base_limits_  = params.limits;

  // Direct laser input: the core is designed for raw scan points. When
  // scan_topic is set, fresh scans feed it and the costmap is only a fallback
  // (stale scan / no scan yet).
  if (!node->has_parameter(name + ".scan_topic"))
  {
    node->declare_parameter<std::string>(name + ".scan_topic", "");
  }
  scan_topic_   = node->get_parameter(name + ".scan_topic").as_string();
  scan_timeout_ = declare_float("scan_timeout", scan_timeout_);
  diagnostics_publish_period_ =
      std::max(0.0f, declare_float("diagnostics_publish_period", diagnostics_publish_period_));
  const std::string downsample_name = name + ".scan_downsample";
  if (!node->has_parameter(downsample_name))
  {
    node->declare_parameter<std::int64_t>(downsample_name, scan_downsample_);
  }
  scan_downsample_ = std::max(1, static_cast<int>(node->get_parameter(downsample_name).as_int()));
  clock_        = node->get_clock();
  if (!scan_topic_.empty())
  {
    auto on_scan = [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      latest_scan_ = msg;
    };
#if BAC_NAV2_API >= 2
    // nav2::LifecycleNode's factory takes the callback before the QoS, which
    // is optional there; the sensor-data profile is kept explicit.
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, on_scan, rclcpp::SensorDataQoS());
#else
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(), on_scan);
#endif
  }
  // Absolute, as diagnostic_updater publishes it: the aggregator listens on
  // /diagnostics whatever namespace the controller server runs in.
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());

  // Costmap-fed points are cell centers, which can sit up to half a cell
  // inside the true obstacle surface. Deduct that quantization error from the
  // safety margins so a wall seen through the costmap does not trigger the
  // emergency stop earlier than the real geometry would (0 disables the
  // compensation). With a direct scan feed the points are exact, so the
  // default is no compensation (the costmap is then only a stale-scan
  // fallback, where uncompensated margins err on the safe side).
  float default_compensation = 0.0f;
  if (scan_topic_.empty())
  {
    if (nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap())
    {
      default_compensation = static_cast<float>(costmap->getResolution()) / 2.0f;
    }
  }
  float compensation         = declare_float("costmap_margin_compensation", default_compensation);
  params.safety_margin.front = std::max(0.05f, params.safety_margin.front - compensation);
  params.safety_margin.rear  = std::max(0.05f, params.safety_margin.rear - compensation);
  params.safety_margin.side  = std::max(0.05f, params.safety_margin.side - compensation);

  {
    const std::string pname = name + ".scan_min_points";
    if (!node->has_parameter(pname))
    {
      node->declare_parameter<std::int64_t>(pname, scan_min_points_);
    }
    scan_min_points_ = std::max(0, static_cast<int>(node->get_parameter(pname).as_int()));
  }
  {
    const std::string pname = name + ".scan_inf_is_valid";
    if (!node->has_parameter(pname))
    {
      node->declare_parameter<bool>(pname, scan_inf_is_valid_);
    }
    scan_inf_is_valid_ = node->get_parameter(pname).as_bool();
  }

  // Who owns the body orientation. "off" (the default) keeps tangent
  // following; "plan" hands the plan's per-pose orientations to the core
  // (BacCore::process path_yaw), so a holonomic vehicle crabs along a path
  // drawn to its side and rotates gradually between two orientations while it
  // travels. Fail closed on anything else, and on a model that steers with
  // yaw: it cannot hold a commanded orientation, and silently ignoring the
  // request would leave the user believing the plan governs when it does not.
  {
    const std::string pname = name + ".plan_yaw_mode";
    if (!node->has_parameter(pname))
    {
      node->declare_parameter<std::string>(pname, "off");
    }
    const std::string mode = node->get_parameter(pname).as_string();
    if (mode == "plan")
    {
      if (params.motion_model.type != MotionModelType::OMNI)
      {
        throw std::invalid_argument(
            "bac: plan_yaw_mode 'plan' requires motion_model.type omni; a model that "
            "steers with yaw cannot hold a commanded orientation");
      }
      plan_yaw_ = true;
    }
    else if (mode != "off")
    {
      throw std::invalid_argument("bac: plan_yaw_mode must be 'off' or 'plan', got '" +
                                  mode + "'");
    }
  }

  // Reject configurations that cannot produce meaningful candidates - a
  // silent nonsense config is worse than a failed configure.
  if (!(params.footprint.width > 0.0f) || !(params.footprint.front > params.footprint.rear))
  {
    throw std::invalid_argument("bac: footprint must have width > 0 and front > rear");
  }
  if (!(params.limits.v_max > 0.0f) || !(params.limits.w_max > 0.0f) ||
      !(params.limits.acc_v > 0.0f))
  {
    throw std::invalid_argument("bac: limits.v_max / w_max / acc_v must be positive");
  }
  if (params.safety_margin.front < 0.0f || params.safety_margin.rear < 0.0f ||
      params.safety_margin.side < 0.0f || params.avoid_margin.side < 0.0f)
  {
    throw std::invalid_argument("bac: margins must be non-negative");
  }
  if (!(params.sim_time > 0.0f) || !(params.stop_decel > 0.0f))
  {
    throw std::invalid_argument("bac: sim_time and stop_decel must be positive");
  }

  core_.setParams(params);

  parameter_callback_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        return onSetParameters(parameters);
      });
}

rcl_interfaces::msg::SetParametersResult
BacController::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (!active_.load())
  {
    return result;
  }
  const std::string prefix = name_ + ".";
  for (const auto &parameter : parameters)
  {
    if (parameter.get_name().compare(0, prefix.size(), prefix) == 0)
    {
      result.successful = false;
      result.reason = "bac: " + parameter.get_name() +
                      " is read when the controller is configured; deactivate, clean up and "
                      "configure the controller server to change it";
      break;
    }
  }
  return result;
}

void
BacController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s", name_.c_str());
  if (parameter_callback_)
  {
    if (auto node = parent_.lock())
    {
      node->remove_on_set_parameters_callback(parameter_callback_.get());
    }
    parameter_callback_.reset();
  }
  scan_sub_.reset();
  diagnostics_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_.reset();
  }
  plan_ = nav_msgs::msg::Path();
  speed_ratio_.store(1.0f);
  core_.reset();
}

void
BacController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s", name_.c_str());
  core_.reset();
  last_diagnostics_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (diagnostics_pub_)
  {
    diagnostics_pub_->on_activate();
  }
  active_.store(true);
}

void
BacController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s", name_.c_str());
  active_.store(false);
  if (diagnostics_pub_)
  {
    diagnostics_pub_->on_deactivate();
  }
}

void
BacController::reset()
{
  core_.reset();
}

void
BacController::acceptPlan(const nav_msgs::msg::Path &path)
{
  plan_ = path;
}

#if BAC_NAV2_API >= 2
void
BacController::newPathReceived(const nav_msgs::msg::Path &raw_global_path)
{
  // The minimal work the interface asks for: keep the plan. It is transformed
  // into the base frame on every tick, in transformPlan().
  acceptPlan(raw_global_path);
}

geometry_msgs::msg::TwistStamped
BacController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::Twist &velocity,
                                       nav2_core::GoalChecker * /*goal_checker*/,
                                       const nav_msgs::msg::Path & /*transformed_global_plan*/,
                                       const geometry_msgs::msg::PoseStamped & /*global_goal*/)
{
  // The path handler's plan is deliberately not used: it lives in the costmap
  // global frame and is cut at the server's prune_distance, while the core's
  // horizon is max_range in the base frame. See the header comment.
  return computeCommand(pose, velocity);
}
#else
void
BacController::setPlan(const nav_msgs::msg::Path &path)
{
  acceptPlan(path);
}

geometry_msgs::msg::TwistStamped
BacController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::Twist &velocity,
                                       nav2_core::GoalChecker * /*goal_checker*/)
{
  return computeCommand(pose, velocity);
}
#endif

std::vector<Point2D>
BacController::collectObstaclePoints(const geometry_msgs::msg::PoseStamped &pose) const
{
  std::vector<Point2D> points;

  nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  double rx = pose.pose.position.x;
  double ry = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  double cs = std::cos(-yaw), sn = std::sin(-yaw);
  double range = core_.params().max_range;

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  if (size_x == 0 || size_y == 0)
  {
    return points;
  }
  // Visit only the cells whose centre can lie within max_range: the square
  // window around the robot, clamped to the map. worldToMapEnforceBounds
  // rounds down, so the window is a superset of the cells the distance test
  // below keeps, and the points come out in the same row-major order as a
  // whole-map walk. A non-finite pose keeps the whole-map walk.
  int mx_lo = 0, my_lo = 0;
  int mx_hi = static_cast<int>(size_x) - 1, my_hi = static_cast<int>(size_y) - 1;
  if (std::isfinite(rx) && std::isfinite(ry) && std::isfinite(range))
  {
    costmap->worldToMapEnforceBounds(rx - range, ry - range, mx_lo, my_lo);
    costmap->worldToMapEnforceBounds(rx + range, ry + range, mx_hi, my_hi);
    mx_lo = std::clamp(mx_lo, 0, static_cast<int>(size_x) - 1);
    my_lo = std::clamp(my_lo, 0, static_cast<int>(size_y) - 1);
    mx_hi = std::clamp(mx_hi, 0, static_cast<int>(size_x) - 1);
    my_hi = std::clamp(my_hi, 0, static_cast<int>(size_y) - 1);
  }
  const unsigned char *charmap = costmap->getCharMap();

  for (unsigned int my = static_cast<unsigned int>(my_lo); my <= static_cast<unsigned int>(my_hi);
       my++)
  {
    const unsigned char *row = charmap + static_cast<std::size_t>(my) * size_x;
    for (unsigned int mx = static_cast<unsigned int>(mx_lo);
         mx <= static_cast<unsigned int>(mx_hi); mx++)
    {
      if (row[mx] < nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }
      double wx, wy;
      costmap->mapToWorld(mx, my, wx, wy);
      double dx = wx - rx, dy = wy - ry;
      if (dx * dx + dy * dy > range * range)
      {
        continue;
      }
      points.emplace_back(static_cast<float>(cs * dx - sn * dy), static_cast<float>(sn * dx + cs * dy));
    }
  }
  return points;
}

std::optional<std::vector<Point2D>>
BacController::collectScanPoints()
{
  if (scan_topic_.empty())
  {
    scan_state_ = "disabled";
    return std::nullopt;
  }
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }
  if (!scan)
  {
    scan_state_ = "not_received";
    return std::nullopt;
  }
  if ((clock_->now() - rclcpp::Time(scan->header.stamp)).seconds() > scan_timeout_)
  {
    scan_state_ = "stale";
    return std::nullopt;
  }

  // Sensor pose in the robot base frame (2D); identity if the scan is already
  // in the base frame.
  double sx = 0.0, sy = 0.0, syaw = 0.0;
  const std::string &base_frame = costmap_ros_->getBaseFrameID();
  if (scan->header.frame_id != base_frame)
  {
    try
    {
      auto tf_msg = tf_->lookupTransform(base_frame, scan->header.frame_id, tf2::TimePointZero);
      sx   = tf_msg.transform.translation.x;
      sy   = tf_msg.transform.translation.y;
      syaw = tf2::getYaw(tf_msg.transform.rotation);
    }
    catch (const tf2::TransformException &)
    {
      scan_state_ = "tf_unavailable";
      return std::nullopt;  // fall back to costmap points
    }
  }

  ScanProjection projected =
      projectScan(scan->ranges, scan->angle_min, scan->angle_increment,
                  scan->range_min, scan->range_max, core_.params().max_range,
                  scan_downsample_, scan_inf_is_valid_, static_cast<float>(sx),
                  static_cast<float>(sy), static_cast<float>(syaw));
  if (projected.valid_ray_count < static_cast<std::size_t>(scan_min_points_))
  {
    // Too few valid MEASUREMENTS (hits or explicit no-returns): a sensor
    // fault, not an empty world - do not treat it as a valid observation.
    scan_state_ = "insufficient_valid_rays";
    return std::nullopt;
  }
  scan_state_ = "fresh";
  return std::move(projected.points);
}

void
BacController::publishDiagnostics(const Result &result, bool using_scan)
{
  if (!diagnostics_pub_ || !diagnostics_pub_->is_activated() ||
      diagnostics_publish_period_ <= 0.0f)
  {
    return;
  }
  const rclcpp::Time stamp = clock_->now();
  const double elapsed = (stamp - last_diagnostics_time_).seconds();
  if (last_diagnostics_time_.nanoseconds() > 0 && elapsed >= 0.0 &&
      elapsed < diagnostics_publish_period_)
  {
    return;
  }
  last_diagnostics_time_ = stamp;

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_ + ": obstacle input and candidate selection";
  status.hardware_id = "none";
  const bool fallback = !scan_topic_.empty() && !using_scan;
  status.level = fallback ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
                            diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = using_scan ? "raw_scan" : (fallback ? "costmap_fallback" : "costmap");

  auto add_value = [&](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(std::move(entry));
  };
  add_value("obstacle_source", status.message);
  add_value("scan_state", scan_state_);
  add_value("bac_status", std::to_string(static_cast<int>(result.status)));
  add_value("candidate_count", std::to_string(result.candidate_count));
  add_value("admissible_count", std::to_string(result.admissible_count));
  add_value("best_clearance_m", std::to_string(result.best_clearance));
  add_value("nearest_distance_m", std::to_string(result.nearest_distance));
  add_value("best_path_cost_m", std::to_string(result.best_path_cost));
  message.status.push_back(std::move(status));
  diagnostics_pub_->publish(message);
}

std::vector<Point2D>
BacController::transformPlan(const geometry_msgs::msg::PoseStamped & /*pose*/,
                             std::optional<float> *goal_heading,
                             std::vector<float> *path_yaw) const
{
  if (goal_heading != nullptr)
  {
    *goal_heading = std::nullopt;
  }
  // Cleared on EVERY exit path, the throws included: the caller reuses the
  // buffer each tick, and a stale sequence would steer the body.
  path_yaw->clear();
  // The plan lives in its own frame (typically map) while the controller runs
  // in the costmap/base frames - subtracting coordinates across frames would
  // silently cancel exactly the localization error this controller is
  // supposed to tolerate. Transform the plan into the BASE frame through TF
  // and treat any frame problem as a hard controller error.
  std::vector<Point2D> path;
  if (plan_.poses.empty())
  {
    return path;
  }
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  const std::string plan_frame = plan_.header.frame_id;
  if (plan_frame.empty())
  {
    throw nav2_core::ControllerTFError("plan has no frame_id");
  }
  geometry_msgs::msg::TransformStamped tf_msg;
  try
  {
    tf_msg = tf_->lookupTransform(base_frame, plan_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    throw nav2_core::ControllerTFError(std::string("cannot transform plan: ") + ex.what());
  }
  std::vector<Point2D> plan_points;
  plan_points.reserve(plan_.poses.size());
  for (const auto &p : plan_.poses)
  {
    plan_points.emplace_back(static_cast<float>(p.pose.position.x),
                             static_cast<float>(p.pose.position.y));
  }
  const float tf_x = static_cast<float>(tf_msg.transform.translation.x);
  const float tf_y = static_cast<float>(tf_msg.transform.translation.y);
  const float tf_yaw = static_cast<float>(tf2::getYaw(tf_msg.transform.rotation));
  // plan_yaw_mode "plan": every plan pose's orientation rides through the
  // prune attached to its own point, then gains the same plan-to-base
  // rotation as the goal heading below. The pruner slices both on one
  // decision; this function never re-states the pruning rule.
  std::vector<float> plan_yaw;
  if (plan_yaw_)
  {
    plan_yaw.reserve(plan_.poses.size());
    for (const auto &p : plan_.poses)
    {
      plan_yaw.push_back(static_cast<float>(tf2::getYaw(p.pose.orientation)));
    }
  }
  std::vector<Point2D> local_path =
      transformAndPrunePath(plan_points, tf_x, tf_y, tf_yaw, core_.params().max_range,
                            plan_yaw_ ? &plan_yaw : nullptr, path_yaw);

  // Nav2 carries the requested goal orientation on the last plan pose. Pass it
  // on only when that pose is still in the pruned path: pruning stops at
  // max_range, and the orientation of an intermediate waypoint is a path
  // tangent, not a goal.
  if (goal_heading != nullptr)
  {
    *goal_heading = goalHeadingInBase(
        plan_points.back(), local_path, tf_x, tf_y, tf_yaw,
        static_cast<float>(tf2::getYaw(plan_.poses.back().pose.orientation)));
  }
  return local_path;
}

geometry_msgs::msg::TwistStamped
BacController::computeCommand(const geometry_msgs::msg::PoseStamped &pose,
                              const geometry_msgs::msg::Twist &velocity)
{
  // The speed limit scales the velocity bounds, yaw rate included. Only a
  // change reaches setParams(); an unchanged limit costs a comparison.
  const float ratio = speed_ratio_.load();
  const Params &current_params = core_.params();
  const float v_max  = base_limits_.v_max * ratio;
  const float v_min  = base_limits_.v_min * ratio;
  const float vy_max = base_limits_.vy_max * ratio;
  const float w_max  = base_limits_.w_max * ratio;
  if (current_params.limits.v_max != v_max || current_params.limits.v_min != v_min ||
      current_params.limits.vy_max != vy_max || current_params.limits.w_max != w_max)
  {
    Params params       = current_params;
    params.limits.v_max  = v_max;
    params.limits.v_min  = v_min;
    params.limits.vy_max = vy_max;
    params.limits.w_max  = w_max;
    core_.setParams(params);
  }

  std::vector<Point2D> points;
  bool using_scan = false;
  if (auto scan_points = collectScanPoints())
  {
    points = std::move(*scan_points);
    using_scan = true;
  }
  else
  {
    points = collectObstaclePoints(pose);
  }
  std::optional<float> goal_heading;
  std::vector<float>   path_yaw;
  std::vector<Point2D> path = transformPlan(pose, &goal_heading, &path_yaw);
  Twist2D current(static_cast<float>(velocity.linear.x), static_cast<float>(velocity.angular.z),
                  static_cast<float>(velocity.linear.y));

  Result result = core_.process(points, path, current, goal_heading, path_yaw);
  publishDiagnostics(result, using_scan);

  // The twist is a body velocity: it is expressed in the base frame, not in
  // the frame of the pose it was computed from.
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp    = clock_->now();
  cmd.twist.linear.x  = result.output.v;
  cmd.twist.linear.y  = result.output.vy;  // zero for every non-holonomic model
  cmd.twist.angular.z = result.output.w;
  return cmd;
}

void
BacController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  if (!std::isfinite(speed_limit) || speed_limit < 0.0)
  {
    RCLCPP_WARN(logger_, "bac: ignoring invalid speed limit %f (keeping the previous one)",
                speed_limit);
    return;
  }
  double ratio = 1.0;  // speed_limit == 0: nav2_costmap_2d::NO_SPEED_LIMIT
  if (speed_limit > 0.0)
  {
    ratio = percentage ? speed_limit / 100.0 :
                         speed_limit / static_cast<double>(base_limits_.v_max);
  }
  // A positive limit never scales a bound to zero: the holonomic and Ackermann
  // models reject a zero limits.w_max, so a limit small enough to underflow
  // the ratio would fail every tick. A millionth of the configured speed is
  // far below the output deadbands, so the robot stands still either way.
  constexpr double kMinSpeedRatio = 1e-6;
  speed_ratio_.store(static_cast<float>(std::clamp(ratio, kMinSpeedRatio, 1.0)));
}

}  // namespace bac

PLUGINLIB_EXPORT_CLASS(bac::BacController, nav2_core::Controller)
