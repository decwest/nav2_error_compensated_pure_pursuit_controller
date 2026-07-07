// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0
//
// Structure follows nav2_regulated_pure_pursuit_controller /
// nav2_dynamic_window_pure_pursuit_controller; only the command-synthesis
// step is replaced by the ECPP curvature law.

#include "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "tf2/utils.hpp"

namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_error_compensated_pure_pursuit_controller
{

void ErrorCompensatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Handles storage and dynamic configuration of the inherited RPP parameters.
  param_handler_ = std::make_unique<rpp::ParameterHandler>(
    node, plugin_name_, logger_, costmap_->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // ECPP-specific parameters
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.omega_n", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.zeta", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.v_epsilon", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.gate_mode", rclcpp::ParameterValue(std::string("ey_only")));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.gate_error_on", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.gate_error_off", rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.gate_endpoint_value", rclcpp::ParameterValue(0.01));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.omega_max", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.v_gain_source", rclcpp::ParameterValue(std::string("commanded")));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.error_search_window", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.error_filter_tau", rclcpp::ParameterValue(0.0));

  std::string gate_mode;
  node->get_parameter(name + ".ecpp.omega_n", ecpp_params_.omega_n);
  node->get_parameter(name + ".ecpp.zeta", ecpp_params_.zeta);
  node->get_parameter(name + ".ecpp.v_epsilon", ecpp_params_.v_epsilon);
  node->get_parameter(name + ".ecpp.gate_mode", gate_mode);
  node->get_parameter(name + ".ecpp.gate_error_on", ecpp_params_.gate_error_on);
  node->get_parameter(name + ".ecpp.gate_error_off", ecpp_params_.gate_error_off);
  node->get_parameter(name + ".ecpp.gate_endpoint_value", ecpp_params_.gate_endpoint_value);
  node->get_parameter(name + ".ecpp.omega_max", ecpp_omega_max_);
  node->get_parameter(name + ".ecpp.v_gain_source", ecpp_v_gain_source_);
  node->get_parameter(name + ".ecpp.error_search_window", ecpp_error_search_window_);
  node->get_parameter(name + ".ecpp.error_filter_tau", e_y_filter_.tau);
  e_psi_filter_.tau = e_y_filter_.tau;
  ecpp_params_.gate_mode = ecpp_math::gateModeFromString(gate_mode);

  if (ecpp_params_.gate_error_off <= ecpp_params_.gate_error_on) {
    throw nav2_core::ControllerException(
            "ecpp.gate_error_off must be greater than ecpp.gate_error_on");
  }
  if (ecpp_params_.gate_endpoint_value <= 0.0 || ecpp_params_.gate_endpoint_value >= 0.5) {
    throw nav2_core::ControllerException("ecpp.gate_endpoint_value must be in (0, 0.5)");
  }
  if (ecpp_params_.omega_n <= 0.0 || ecpp_params_.zeta <= 0.0 || ecpp_params_.v_epsilon <= 0.0) {
    throw nav2_core::ControllerException("ecpp.omega_n, zeta, and v_epsilon must be positive");
  }
  if (e_y_filter_.tau < 0.0) {
    throw nav2_core::ControllerException("ecpp.error_filter_tau must be non-negative");
  }

  // Handles global path transformations
  path_handler_ = std::make_unique<rpp::PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<rpp::CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "curvature_lookahead_point", 1);
  is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "is_rotating_to_heading", 1);
  ecpp_debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    plugin_name_ + "/ecpp_debug", 1);

  ecpp_dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ErrorCompensatedPurePursuitController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

namespace
{
double calculateCurvature(const geometry_msgs::msg::Point & lookahead_point)
{
  // Chord length^2 to the lookahead point (carrot) in the robot base frame
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  }
  return 0.0;
}
}  // namespace

geometry_msgs::msg::TwistStamped ErrorCompensatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
  global_path_pub_->publish(transformed_plan);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);
  double curv_lookahead_dist = params_->curvature_lookahead_dist;

  // Check for reverse driving
  if (params_->allow_reversing) {
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
    if (dist_to_cusp < curv_lookahead_dist) {
      curv_lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  auto curvature_carrot_position = carrot_pose.pose.position;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(
      curv_lookahead_dist,
      transformed_plan, params_->interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    curvature_carrot_position = curvature_lookahead_pose.pose.position;
    curvature_carrot_pub_->publish(createCarrotMsg(curvature_lookahead_pose));
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (params_->allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->desired_linear_vel;

  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    is_rotating_to_heading_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign)) {
    is_rotating_to_heading_ = true;
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    is_rotating_to_heading_ = false;
    applyConstraints(
      regulation_curvature, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, x_vel_sign);

    if (cancelling_) {
      const double & dt = control_duration_;
      linear_vel = speed.linear.x - x_vel_sign * dt * params_->cancel_deceleration;

      if (x_vel_sign > 0) {
        if (linear_vel <= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      } else {
        if (linear_vel >= 0) {
          linear_vel = 0;
          finished_cancelling_ = true;
        }
      }
    }

    // Conventional (regulated) pure pursuit would now apply
    //   angular_vel = linear_vel * regulation_curvature;
    // ECPP instead compensates the pure pursuit curvature with gated
    // difference-gain error feedback in the path frame.
    const auto path_error = ecpp_math::computePathFrameError(
      transformed_plan, ecpp_error_search_window_);

    // L_d for the PP-equivalent gains must match the curvature actually used:
    // kappa_pp = 2 y_b / l^2 with l the chord to the carrot, so the intrinsic
    // pure pursuit gains are 2/l^2 and 2/l.
    const double l_d_eff = std::max(
      std::hypot(curvature_carrot_position.x, curvature_carrot_position.y), 1e-3);
    const double v_for_gain =
      (ecpp_v_gain_source_ == "measured") ?
      std::fabs(speed.linear.x) : std::fabs(linear_vel);

    ecpp_math::EcppParams active_params = ecpp_params_;
    if (!path_error.valid || x_vel_sign < 0.0) {
      // Degenerate plan or reversing: fall back to plain pure pursuit.
      active_params.gate_mode = ecpp_math::GateMode::OFF;
      e_y_filter_.reset();
      e_psi_filter_.reset();
      if (x_vel_sign < 0.0) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "ECPP compensation disabled while reversing (forward-only design).");
      }
    }

    // Standard derivative-style filtering of the error signals used by the
    // compensation (ecpp.error_filter_tau, 0 = off). The pure pursuit base
    // curvature stays unfiltered, preserving its capture behavior.
    const double e_y_used = e_y_filter_.update(path_error.e_y, control_duration_);
    const double e_psi_used = e_psi_filter_.update(path_error.e_psi, control_duration_);

    const auto terms = ecpp_math::computeEcppTerms(
      regulation_curvature, e_y_used, e_psi_used,
      l_d_eff, v_for_gain, active_params);

    angular_vel = std::clamp(
      linear_vel * terms.curvature, -ecpp_omega_max_, ecpp_omega_max_);

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
      terms.e_y, terms.e_psi, terms.sigma, terms.sigma_y, terms.sigma_psi,
      terms.kappa_pp, terms.curvature, terms.v_gain, terms.lookahead_dist};
    ecpp_debug_pub_->publish(debug_msg);
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    throw nav2_core::NoValidControl(
            "ErrorCompensatedPurePursuitController detected collision ahead!");
  }

  // Publish whether we are rotating to goal heading
  std_msgs::msg::Bool is_rotating_to_heading_msg;
  is_rotating_to_heading_msg.data = is_rotating_to_heading_;
  is_rotating_to_heading_pub_->publish(is_rotating_to_heading_msg);

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  last_command_velocity_ = cmd_vel.twist;

  return cmd_vel;
}

rcl_interfaces::msg::SetParametersResult
ErrorCompensatedPurePursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  const std::string prefix = plugin_name_ + ".ecpp.";

  ecpp_math::EcppParams new_params = ecpp_params_;
  double new_omega_max = ecpp_omega_max_;
  double new_search_window = ecpp_error_search_window_;
  double new_filter_tau = e_y_filter_.tau;
  std::string new_v_gain_source = ecpp_v_gain_source_;

  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string field = name.substr(prefix.size());
    try {
      if (field == "omega_n") {
        new_params.omega_n = parameter.as_double();
      } else if (field == "zeta") {
        new_params.zeta = parameter.as_double();
      } else if (field == "v_epsilon") {
        new_params.v_epsilon = parameter.as_double();
      } else if (field == "gate_mode") {
        new_params.gate_mode = ecpp_math::gateModeFromString(parameter.as_string());
      } else if (field == "gate_error_on") {
        new_params.gate_error_on = parameter.as_double();
      } else if (field == "gate_error_off") {
        new_params.gate_error_off = parameter.as_double();
      } else if (field == "gate_endpoint_value") {
        new_params.gate_endpoint_value = parameter.as_double();
      } else if (field == "omega_max") {
        new_omega_max = parameter.as_double();
      } else if (field == "v_gain_source") {
        new_v_gain_source = parameter.as_string();
      } else if (field == "error_search_window") {
        new_search_window = parameter.as_double();
      } else if (field == "error_filter_tau") {
        new_filter_tau = parameter.as_double();
      }
    } catch (const std::exception & ex) {
      result.successful = false;
      result.reason = ex.what();
      return result;
    }
  }

  if (new_params.omega_n <= 0.0 || new_params.zeta <= 0.0 || new_params.v_epsilon <= 0.0) {
    result.successful = false;
    result.reason = "ecpp.omega_n, zeta, and v_epsilon must be positive";
    return result;
  }
  if (new_params.gate_error_off <= new_params.gate_error_on) {
    result.successful = false;
    result.reason = "ecpp.gate_error_off must be greater than ecpp.gate_error_on";
    return result;
  }
  if (new_params.gate_endpoint_value <= 0.0 || new_params.gate_endpoint_value >= 0.5) {
    result.successful = false;
    result.reason = "ecpp.gate_endpoint_value must be in (0, 0.5)";
    return result;
  }
  if (new_v_gain_source != "commanded" && new_v_gain_source != "measured") {
    result.successful = false;
    result.reason = "ecpp.v_gain_source must be 'commanded' or 'measured'";
    return result;
  }
  if (new_filter_tau < 0.0) {
    result.successful = false;
    result.reason = "ecpp.error_filter_tau must be non-negative";
    return result;
  }

  ecpp_params_ = new_params;
  ecpp_omega_max_ = new_omega_max;
  ecpp_error_search_window_ = new_search_window;
  if (new_filter_tau != e_y_filter_.tau) {
    e_y_filter_.tau = new_filter_tau;
    e_psi_filter_.tau = new_filter_tau;
    e_y_filter_.reset();
    e_psi_filter_.reset();
  }
  ecpp_v_gain_source_ = new_v_gain_source;
  return result;
}

void ErrorCompensatedPurePursuitController::activate()
{
  RegulatedPurePursuitController::activate();
  ecpp_debug_pub_->on_activate();
  e_y_filter_.reset();
  e_psi_filter_.reset();
}

void ErrorCompensatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  RegulatedPurePursuitController::setPlan(path);
  e_y_filter_.reset();
  e_psi_filter_.reset();
}

void ErrorCompensatedPurePursuitController::deactivate()
{
  RegulatedPurePursuitController::deactivate();
  ecpp_debug_pub_->on_deactivate();
  last_command_velocity_ = geometry_msgs::msg::Twist();
}

void ErrorCompensatedPurePursuitController::cleanup()
{
  RegulatedPurePursuitController::cleanup();
  ecpp_debug_pub_.reset();
  ecpp_dyn_params_handler_.reset();
}

void ErrorCompensatedPurePursuitController::reset()
{
  cancelling_ = false;
  finished_cancelling_ = false;
  has_reached_xy_tolerance_ = false;
  last_command_velocity_ = geometry_msgs::msg::Twist();
  e_y_filter_.reset();
  e_psi_filter_.reset();
}

}  // namespace nav2_error_compensated_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_error_compensated_pure_pursuit_controller::ErrorCompensatedPurePursuitController,
  nav2_core::Controller)
