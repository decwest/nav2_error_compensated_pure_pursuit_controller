// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2026 Fumiya Ohnishi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modified for continuous arc-length lookahead and ECPP curvature compensation.
//
// Extends nav2_regulated_pure_pursuit_controller with an arc-length carrot,
// gated ECPP curvature compensation.

#include \
  "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "tf2/utils.hpp"

#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"

namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_error_compensated_pure_pursuit_controller
{

namespace
{

std::string validateEcppParameters(
  const ecpp_math::EcppParams & params,
  const double desired_linear_vel,
  const double max_angular_accel,
  const bool use_fixed_curvature_lookahead,
  const std::string & v_gain_source)
{
  if (!std::isfinite(params.omega_n) || !std::isfinite(params.zeta) ||
    !std::isfinite(params.v_epsilon) || !std::isfinite(params.gate_error_on) ||
    !std::isfinite(params.gate_error_off) || !std::isfinite(params.gate_endpoint_value) ||
    !std::isfinite(desired_linear_vel) || !std::isfinite(max_angular_accel))
  {
    return "ECPP numeric parameters must be finite";
  }
  if (params.omega_n <= 0.0 || params.zeta <= 0.0 || params.v_epsilon <= 0.0) {
    return "ecpp.omega_n, zeta, and v_epsilon must be positive";
  }
  if (params.gate_error_on < 0.0) {
    return "ecpp.gate_error_on must be non-negative";
  }
  if (params.gate_error_off <= params.gate_error_on) {
    return "ecpp.gate_error_off must be greater than ecpp.gate_error_on";
  }
  if (params.gate_endpoint_value <= 0.0 || params.gate_endpoint_value >= 0.5) {
    return "ecpp.gate_endpoint_value must be in (0, 0.5)";
  }
  if (desired_linear_vel <= 0.0) {
    return "desired_linear_vel must be positive";
  }
  if (max_angular_accel < 0.0) {
    return "max_angular_accel must be non-negative";
  }
  if (params.gate_mode != ecpp_math::GateMode::OFF && use_fixed_curvature_lookahead) {
    return "ECPP requires use_fixed_curvature_lookahead=false so that compensation and "
           "pure pursuit use the same nominal lookahead";
  }
  if (v_gain_source != "commanded" && v_gain_source != "measured") {
    return "ecpp.v_gain_source must be 'commanded' or 'measured'";
  }
  return "";
}

struct ControllerParameterState
{
  ecpp_math::EcppParams ecpp_params;
  bool use_fixed_curvature_lookahead;
  double desired_linear_vel;
  double max_angular_accel;
  std::string v_gain_source;
  bool publish_debug;
};

rcl_interfaces::msg::SetParametersResult stageParameterUpdates(
  const std::vector<rclcpp::Parameter> & parameters,
  const std::string & plugin_name,
  ControllerParameterState & state)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const std::string plugin_prefix = plugin_name + ".";
  const std::string ecpp_prefix = plugin_prefix + "ecpp.";

  const auto require_type = [&result](
    const rclcpp::Parameter & parameter,
    const rclcpp::ParameterType expected_type)
    {
      if (parameter.get_type() == expected_type) {
        return true;
      }
      result.successful = false;
      result.reason = "Invalid type for parameter '" + parameter.get_name() + "'";
      return false;
    };

  for (const auto & parameter : parameters) {
    const std::string & parameter_name = parameter.get_name();
    if (parameter_name.rfind(ecpp_prefix, 0) == 0) {
      const std::string field = parameter_name.substr(ecpp_prefix.size());
      if (field == "publish_debug") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_BOOL)) {
          return result;
        }
        state.publish_debug = parameter.as_bool();
      } else if (field == "omega_n") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_params.omega_n = parameter.as_double();
      } else if (field == "zeta") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_params.zeta = parameter.as_double();
      } else if (field == "v_epsilon") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_params.v_epsilon = parameter.as_double();
      } else if (field == "gate_mode") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_STRING)) {
          return result;
        }
        try {
          state.ecpp_params.gate_mode = ecpp_math::gateModeFromString(parameter.as_string());
        } catch (const std::invalid_argument & exception) {
          result.successful = false;
          result.reason = exception.what();
          return result;
        }
      } else if (field == "gate_error_on") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_params.gate_error_on = parameter.as_double();
      } else if (field == "gate_error_off") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_params.gate_error_off = parameter.as_double();
      } else if (field == "gate_endpoint_value") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_params.gate_endpoint_value = parameter.as_double();
      } else if (field == "v_gain_source") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_STRING)) {
          return result;
        }
        state.v_gain_source = parameter.as_string();
      }
      continue;
    }

    if (parameter_name == plugin_prefix + "use_fixed_curvature_lookahead") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_BOOL)) {
        return result;
      }
      state.use_fixed_curvature_lookahead = parameter.as_bool();
    } else if (parameter_name == plugin_prefix + "desired_linear_vel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.desired_linear_vel = parameter.as_double();
    } else if (parameter_name == plugin_prefix + "max_angular_accel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.max_angular_accel = parameter.as_double();
    }
  }

  return result;
}

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

  // ECPP-specific parameters.
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.publish_debug", rclcpp::ParameterValue(false));
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
    node.get(), name + ".ecpp.v_gain_source", rclcpp::ParameterValue(std::string("commanded")));

  std::string gate_mode;
  node->get_parameter(name + ".ecpp.publish_debug", ecpp_publish_debug_);
  node->get_parameter(name + ".ecpp.omega_n", ecpp_params_.omega_n);
  node->get_parameter(name + ".ecpp.zeta", ecpp_params_.zeta);
  node->get_parameter(name + ".ecpp.v_epsilon", ecpp_params_.v_epsilon);
  node->get_parameter(name + ".ecpp.gate_mode", gate_mode);
  node->get_parameter(name + ".ecpp.gate_error_on", ecpp_params_.gate_error_on);
  node->get_parameter(name + ".ecpp.gate_error_off", ecpp_params_.gate_error_off);
  node->get_parameter(name + ".ecpp.gate_endpoint_value", ecpp_params_.gate_endpoint_value);
  node->get_parameter(name + ".ecpp.v_gain_source", ecpp_v_gain_source_);

  try {
    ecpp_params_.gate_mode = ecpp_math::gateModeFromString(gate_mode);
  } catch (const std::invalid_argument & exception) {
    throw nav2_core::ControllerException(exception.what());
  }

  const std::string ecpp_validation_error = validateEcppParameters(
    ecpp_params_, params_->base_desired_linear_vel, params_->max_angular_accel,
    params_->use_fixed_curvature_lookahead, ecpp_v_gain_source_);
  if (!ecpp_validation_error.empty()) {
    throw nav2_core::ControllerException(ecpp_validation_error);
  }
  // Plugin-local transformer retains the predecessor of the projection segment without changing
  // Navigation2's shared RPP implementation.
  ecpp_path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_, costmap_ros_);

  // Checks for imminent collisions
  collision_checker_ = std::make_unique<rpp::CollisionChecker>(node, costmap_ros_, params_);

  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  node->get_parameter("controller_frequency", control_frequency);
  if (!std::isfinite(control_frequency) || control_frequency <= 0.0) {
    throw nav2_core::ControllerException("controller_frequency must be finite and positive");
  }
  control_duration_ = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "curvature_lookahead_point", 1);
  is_rotating_to_heading_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "is_rotating_to_heading", 1);
  ecpp_debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    plugin_name_ + "/ecpp_debug", 1);

  parameter_validation_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ErrorCompensatedPurePursuitController::validateParameterUpdates,
      this, std::placeholders::_1));
  parameter_update_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ErrorCompensatedPurePursuitController::updateParameters,
      this, std::placeholders::_1));
}

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

  // Select the projection once and transform it together with the path to the robot base frame.
  // Pruning, carrot, path-frame errors, and cusp search all share this progress choice.
  const auto transformed = ecpp_path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist, params_->interpolate_curvature_after_goal);
  const auto & transformed_plan = transformed.path;
  const auto & path_projection = transformed.projection;
  global_path_pub_->publish(transformed_plan);

  // Find look ahead distance and point on path and publish. The nominal distance is retained for
  // the PP-equivalent ECPP gains even if the carrot distance is shortened at a reversing cusp.
  const double nominal_lookahead_dist = getLookAheadDistance(speed);
  double lookahead_dist = nominal_lookahead_dist;
  double curv_lookahead_dist = params_->curvature_lookahead_dist;

  // Check for reverse driving
  if (params_->allow_reversing) {
    const double dist_to_cusp =
      arc_length_lookahead::findVelocitySignChangeArcLength(transformed_plan, path_projection);
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
    if (dist_to_cusp < curv_lookahead_dist) {
      curv_lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = arc_length_lookahead::getLookAheadPoint(
    lookahead_dist, transformed_plan, path_projection);
  auto rotate_to_path_carrot_pose = carrot_pose;
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);

  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = arc_length_lookahead::getLookAheadPoint(
      curv_lookahead_dist, transformed_plan, path_projection);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
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

    ecpp_math::EcppTerms terms;
    terms.kappa_pp = regulation_curvature;
    terms.curvature = regulation_curvature;
    terms.lookahead_dist = nominal_lookahead_dist;

    if (ecpp_params_.gate_mode != ecpp_math::GateMode::OFF) {
      const auto path_error = ecpp_math::computePathFrameError(
        path_projection);
      const double v_for_gain =
        (ecpp_v_gain_source_ == "measured") ?
        std::fabs(speed.linear.x) : std::fabs(linear_vel);

      ecpp_math::EcppParams active_params = ecpp_params_;
      if (!path_error.valid || x_vel_sign < 0.0) {
        // Degenerate plan or reversing: fall back to plain pure pursuit.
        active_params.gate_mode = ecpp_math::GateMode::OFF;
        if (x_vel_sign < 0.0) {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 5000,
            "ECPP compensation disabled while reversing (forward-only design).");
        }
      }

      terms = ecpp_math::computeEcppTerms(
        regulation_curvature, path_error.e_y, path_error.e_psi,
        nominal_lookahead_dist, v_for_gain, params_->base_desired_linear_vel, active_params);
      if (path_projection.valid &&
        path_projection.remaining_arc_length < nominal_lookahead_dist)
      {
        RCLCPP_DEBUG_THROTTLE(
          logger_, *clock_, 5000,
          "Remaining path arc length (%.3f m) is shorter than nominal ECPP lookahead "
          "(%.3f m); local gain guarantee does not apply in this endpoint region.",
          path_projection.remaining_arc_length, nominal_lookahead_dist);
      }
    }

    regulation_curvature = terms.curvature;
    angular_vel = linear_vel * regulation_curvature;

    if (ecpp_publish_debug_) {
      std_msgs::msg::Float64MultiArray debug_msg;
      debug_msg.data = {
        terms.e_y, terms.e_psi, terms.sigma, terms.sigma_y, terms.sigma_psi,
        terms.kappa_pp, regulation_curvature, terms.v_gain, terms.lookahead_dist};
      ecpp_debug_pub_->publish(debug_msg);
    }
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

  return cmd_vel;
}

rcl_interfaces::msg::SetParametersResult
ErrorCompensatedPurePursuitController::validateParameterUpdates(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  ControllerParameterState state{
    ecpp_params_, params_->use_fixed_curvature_lookahead, params_->base_desired_linear_vel,
    params_->max_angular_accel, ecpp_v_gain_source_, ecpp_publish_debug_};
  auto result = stageParameterUpdates(
    parameters, plugin_name_, state);
  if (!result.successful) {
    return result;
  }

  result.reason = validateEcppParameters(
    state.ecpp_params, state.desired_linear_vel, state.max_angular_accel,
    state.use_fixed_curvature_lookahead, state.v_gain_source);
  result.successful = result.reason.empty();
  return result;
}

void ErrorCompensatedPurePursuitController::updateParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  ControllerParameterState state{
    ecpp_params_, params_->use_fixed_curvature_lookahead, params_->base_desired_linear_vel,
    params_->max_angular_accel, ecpp_v_gain_source_, ecpp_publish_debug_};
  const auto result = stageParameterUpdates(
    parameters, plugin_name_, state);
  if (!result.successful) {
    RCLCPP_ERROR(logger_, "Validated parameter update could not be applied: %s",
        result.reason.c_str());
    return;
  }

  ecpp_publish_debug_ = state.publish_debug;
  ecpp_params_ = state.ecpp_params;
  ecpp_v_gain_source_ = state.v_gain_source;
}

void ErrorCompensatedPurePursuitController::activate()
{
  RegulatedPurePursuitController::activate();
  ecpp_debug_pub_->on_activate();
}

void ErrorCompensatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  has_reached_xy_tolerance_ = false;
  ecpp_path_handler_->setPlan(path);
}

void ErrorCompensatedPurePursuitController::deactivate()
{
  RegulatedPurePursuitController::deactivate();
  ecpp_debug_pub_->on_deactivate();
}

void ErrorCompensatedPurePursuitController::cleanup()
{
  auto node = node_.lock();
  if (node && parameter_validation_handler_) {
    node->remove_on_set_parameters_callback(parameter_validation_handler_.get());
  }
  if (node && parameter_update_handler_) {
    node->remove_post_set_parameters_callback(parameter_update_handler_.get());
  }
  parameter_validation_handler_.reset();
  parameter_update_handler_.reset();
  RegulatedPurePursuitController::cleanup();
  ecpp_path_handler_.reset();
  ecpp_debug_pub_.reset();
}

void ErrorCompensatedPurePursuitController::reset()
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  cancelling_ = false;
  finished_cancelling_ = false;
  has_reached_xy_tolerance_ = false;
}

}  // namespace nav2_error_compensated_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_error_compensated_pure_pursuit_controller::ErrorCompensatedPurePursuitController,
  nav2_core::Controller)
