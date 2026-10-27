// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0
//
// Extends nav2_regulated_pure_pursuit_controller with an arc-length carrot,
// optional ECPP curvature compensation, and the official DWPP speed synthesis.

#include \
  "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "tf2/utils.hpp"

#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"

namespace rpp = nav2_regulated_pure_pursuit_controller;

using nav2::declare_parameter_if_not_declared;
namespace dynamic_window_pure_pursuit = rpp::dynamic_window_pure_pursuit;

namespace nav2_error_compensated_pure_pursuit_controller
{

namespace
{

std::string validateEcppParameters(
  const ecpp_math::EcppParams & params,
  const double omega_max,
  const double error_search_window,
  const double error_filter_tau,
  const std::string & v_gain_source)
{
  if (!std::isfinite(params.omega_n) || !std::isfinite(params.zeta) ||
    !std::isfinite(params.v_epsilon) || !std::isfinite(params.gate_error_on) ||
    !std::isfinite(params.gate_error_off) || !std::isfinite(params.gate_endpoint_value) ||
    !std::isfinite(omega_max) || !std::isfinite(error_search_window) ||
    !std::isfinite(error_filter_tau))
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
  if (omega_max <= 0.0) {
    return "ecpp.omega_max must be positive";
  }
  if (error_search_window <= 0.0) {
    return "ecpp.error_search_window must be positive";
  }
  if (error_filter_tau < 0.0) {
    return "ecpp.error_filter_tau must be non-negative";
  }
  if (v_gain_source != "commanded" && v_gain_source != "measured") {
    return "ecpp.v_gain_source must be 'commanded' or 'measured'";
  }
  return "";
}

std::string validateDynamicWindowParameters(
  const rpp::Parameters & params,
  const double max_linear_vel,
  const double max_angular_accel)
{
  if (!std::isfinite(max_linear_vel) || !std::isfinite(params.min_linear_vel) ||
    !std::isfinite(params.max_angular_vel) || !std::isfinite(params.min_angular_vel) ||
    !std::isfinite(params.max_linear_accel) || !std::isfinite(params.max_linear_decel) ||
    !std::isfinite(max_angular_accel) || !std::isfinite(params.max_angular_decel))
  {
    return "Dynamic window velocity and acceleration parameters must be finite";
  }
  if (max_linear_vel < 0.0) {
    return "max_linear_vel must be non-negative";
  }
  if (params.min_linear_vel > max_linear_vel) {
    return "min_linear_vel must be no greater than max_linear_vel";
  }
  if (params.max_angular_vel < 0.0) {
    return "max_angular_vel must be non-negative";
  }
  if (params.min_angular_vel > params.max_angular_vel) {
    return "min_angular_vel must be no greater than max_angular_vel";
  }
  if (params.max_linear_accel < 0.0 || max_angular_accel < 0.0) {
    return "max_linear_accel and max_angular_accel must be non-negative";
  }
  if (params.max_linear_decel > 0.0 || params.max_angular_decel > 0.0) {
    return "max_linear_decel and max_angular_decel must be non-positive";
  }
  return "";
}

struct ControllerParameterState
{
  bool use_error_compensation;
  bool use_fixed_curvature_lookahead;
  ecpp_math::EcppParams ecpp_params;
  double ecpp_omega_max;
  double error_search_window;
  double error_filter_tau;
  std::string v_gain_source;
  rpp::Parameters dynamic_window_params;  // Validation snapshot; storage is owned by RPP.
  double max_linear_vel;
  double max_angular_accel;
};

rcl_interfaces::msg::SetParametersResult stageParameterUpdates(
  const std::vector<rclcpp::Parameter> & parameters,
  const std::string & plugin_name,
  const bool use_legacy_omega_max_alias,
  ControllerParameterState & state)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const std::string plugin_prefix = plugin_name + ".";
  const std::string ecpp_prefix = plugin_prefix + "ecpp.";
  bool legacy_omega_max_updated = false;
  bool max_angular_vel_updated = false;

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
      if (field == "use_error_compensation") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_BOOL)) {
          return result;
        }
        state.use_error_compensation = parameter.as_bool();
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
      } else if (field == "omega_max") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.ecpp_omega_max = parameter.as_double();
        legacy_omega_max_updated = true;
      } else if (field == "v_gain_source") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_STRING)) {
          return result;
        }
        state.v_gain_source = parameter.as_string();
      } else if (field == "error_search_window") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.error_search_window = parameter.as_double();
      } else if (field == "error_filter_tau") {
        if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
          return result;
        }
        state.error_filter_tau = parameter.as_double();
      }
      continue;
    }

    if (parameter_name == plugin_prefix + "use_dynamic_window") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_BOOL)) {
        return result;
      }
      state.dynamic_window_params.use_dynamic_window = parameter.as_bool();
    } else if (parameter_name == plugin_prefix + "use_fixed_curvature_lookahead") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_BOOL)) {
        return result;
      }
      state.use_fixed_curvature_lookahead = parameter.as_bool();
    } else if (parameter_name == plugin_prefix + "min_linear_vel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.dynamic_window_params.min_linear_vel = parameter.as_double();
    } else if (parameter_name == plugin_prefix + "max_angular_vel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.dynamic_window_params.max_angular_vel = parameter.as_double();
      max_angular_vel_updated = true;
    } else if (parameter_name == plugin_prefix + "min_angular_vel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.dynamic_window_params.min_angular_vel = parameter.as_double();
    } else if (parameter_name == plugin_prefix + "max_linear_accel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.dynamic_window_params.max_linear_accel = parameter.as_double();
    } else if (parameter_name == plugin_prefix + "max_linear_decel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.dynamic_window_params.max_linear_decel = parameter.as_double();
    } else if (parameter_name == plugin_prefix + "max_angular_decel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.dynamic_window_params.max_angular_decel = parameter.as_double();
    } else if ((parameter_name == plugin_prefix + "max_linear_vel" ||
      parameter_name == plugin_prefix + "desired_linear_vel")) {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.max_linear_vel = parameter.as_double();
    } else if (parameter_name == plugin_prefix + "max_angular_accel") {
      if (!require_type(parameter, rclcpp::ParameterType::PARAMETER_DOUBLE)) {
        return result;
      }
      state.max_angular_accel = parameter.as_double();
    }
  }

  // Only a startup configuration containing the legacy name without the canonical name enables
  // aliasing. Otherwise changing ecpp.omega_max must not desynchronize the declared canonical
  // max_angular_vel parameter from the controller state.
  if (use_legacy_omega_max_alias && legacy_omega_max_updated && !max_angular_vel_updated) {
    state.dynamic_window_params.max_angular_vel = state.ecpp_omega_max;
  }

  if (state.use_error_compensation && state.use_fixed_curvature_lookahead) {
    result.successful = false;
    result.reason =
      "ECPP cannot be combined with use_fixed_curvature_lookahead=true because the "
      "compensation and pure-pursuit base must use the same nominal lookahead";
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
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  const bool has_legacy_omega_max_override = overrides.count(name + ".ecpp.omega_max") != 0;
  use_legacy_omega_max_alias_ =
    has_legacy_omega_max_override && overrides.count(name + ".max_angular_vel") == 0;
  if (use_legacy_omega_max_alias_) {
    declare_parameter_if_not_declared(
      node.get(), name + ".max_angular_vel", overrides.at(name + ".ecpp.omega_max"));
  }

  // Lyrical owns the complete RPP/DWPP parameter set and collision checker.
  RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);
  clock_ = node->get_clock();
  max_robot_pose_search_dist_ = node->declare_or_get_parameter(
    name + ".max_robot_pose_search_dist", costmap_->getSizeInMetersX() / 2.0);
  if (!std::isfinite(max_robot_pose_search_dist_) || max_robot_pose_search_dist_ <= 0.0) {
    throw nav2_core::ControllerException("max_robot_pose_search_dist must be finite and positive");
  }

  // ECPP-specific parameters.
  declare_parameter_if_not_declared(
    node.get(), name + ".ecpp.use_error_compensation", rclcpp::ParameterValue(false));
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
  node->get_parameter(name + ".ecpp.use_error_compensation", use_error_compensation_);
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

  if (has_legacy_omega_max_override) {
    RCLCPP_WARN(
      logger_,
      "Parameter '%s.ecpp.omega_max' is deprecated and no longer clamps non-DWPP commands. "
      "Use '%s.max_angular_vel' with use_dynamic_window instead.",
      name.c_str(), name.c_str());
  }

  try {
    ecpp_params_.gate_mode = ecpp_math::gateModeFromString(gate_mode);
  } catch (const std::invalid_argument & exception) {
    throw nav2_core::ControllerException(exception.what());
  }

  const std::string ecpp_validation_error = validateEcppParameters(
    ecpp_params_, ecpp_omega_max_, ecpp_error_search_window_, e_y_filter_.tau,
    ecpp_v_gain_source_);
  if (!ecpp_validation_error.empty()) {
    throw nav2_core::ControllerException(ecpp_validation_error);
  }
  const std::string dynamic_window_validation_error = validateDynamicWindowParameters(
    *params_, params_->max_linear_vel, params_->max_angular_accel);
  if (!dynamic_window_validation_error.empty()) {
    throw nav2_core::ControllerException(dynamic_window_validation_error);
  }
  if (use_error_compensation_ && params_->use_fixed_curvature_lookahead) {
    throw nav2_core::ControllerException(
            "ECPP cannot be combined with use_fixed_curvature_lookahead=true because the "
            "compensation and pure-pursuit base must use the same nominal lookahead");
  }

  // Plugin-local transformer retains the predecessor of the projection segment without changing
  // Navigation2's shared RPP implementation.
  ecpp_path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(costmap_ros_->getTransformTolerance()), tf_, costmap_ros_);

  double control_frequency = 20.0;
  node->get_parameter("controller_frequency", control_frequency);
  if (!std::isfinite(control_frequency) || control_frequency <= 0.0) {
    throw nav2_core::ControllerException("controller_frequency must be finite and positive");
  }
  control_duration_ = 1.0 / control_frequency;
  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan");
  ecpp_debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    plugin_name_ + "/ecpp_debug");
  if (use_legacy_omega_max_alias_) {
    legacy_parameter_handler_ = node->add_pre_set_parameters_callback(
      [this](std::vector<rclcpp::Parameter> & parameters) {
        const auto canonical = plugin_name_ + ".max_angular_vel";
        if (std::any_of(parameters.begin(), parameters.end(),
          [&canonical](const auto & p) {return p.get_name() == canonical;}))
        {
          return;
        }
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == plugin_name_ + ".ecpp.omega_max" &&
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            const double value = parameter.as_double();
            parameters.emplace_back(canonical, value);
            break;
          }
        }
      });
  }

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
  nav2_core::GoalChecker * goal_checker,
  const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  if (transformed_global_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Controller server supplied an empty path");
  }

  // Transform path to robot base frame
  auto transformed_plan = ecpp_path_handler_->transformGlobalPlan(
    pose, max_robot_pose_search_dist_, params_->interpolate_curvature_after_goal);
  global_path_pub_->publish(transformed_plan);

  // Project exactly once per cycle. Carrot, path-frame errors, and cusp search all consume this
  // same result, so progress cannot disagree at corners or self intersections.
  const auto path_projection = arc_length_lookahead::projectPath(
    transformed_plan, ecpp_error_search_window_);

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

  linear_vel = params_->max_linear_vel;

  double angle_to_heading;
  if (shouldRotateToGoalHeading(
      goal_checker, pose, global_goal, speed, transformed_global_plan)) {
    is_rotating_to_heading_ = true;
    geometry_msgs::msg::PoseStamped goal_in_base;
    if (!ecpp_path_handler_->transformPose(
        costmap_ros_->getBaseFrameID(), global_goal, goal_in_base))
    {
      throw nav2_core::ControllerTFError("Unable to transform final goal into robot frame");
    }
    double angle_to_goal = tf2::getYaw(goal_in_base.pose.orientation);
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

    if (use_error_compensation_) {
      const auto path_error = ecpp_math::computePathFrameError(
        path_projection);
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

      // Filter only the error signals used by the compensation. The PP base curvature remains
      // unfiltered so that its geometric capture behavior is unchanged.
      const double e_y_used = e_y_filter_.update(path_error.e_y, control_duration_);
      const double e_psi_used = e_psi_filter_.update(path_error.e_psi, control_duration_);

      terms = ecpp_math::computeEcppTerms(
        regulation_curvature, e_y_used, e_psi_used,
        nominal_lookahead_dist, v_for_gain, active_params);
      if (path_projection.valid &&
        path_projection.remaining_arc_length < nominal_lookahead_dist)
      {
        RCLCPP_DEBUG_THROTTLE(
          logger_, *clock_, 5000,
          "Remaining path arc length (%.3f m) is shorter than nominal ECPP lookahead "
          "(%.3f m); local gain guarantee does not apply in this endpoint region.",
          path_projection.remaining_arc_length, nominal_lookahead_dist);
      }
    } else {
      e_y_filter_.reset();
      e_psi_filter_.reset();
    }

    regulation_curvature = ecpp_math::selectRegulationCurvature(
      terms, use_error_compensation_);

    if (!params_->use_dynamic_window) {
      angular_vel = linear_vel * regulation_curvature;
    } else {
      std::tie(linear_vel, angular_vel) =
        dynamic_window_pure_pursuit::computeDynamicWindowVelocities(
        last_command_velocity_, params_->max_linear_vel,
        params_->min_linear_vel,
        params_->max_angular_vel,
        params_->min_angular_vel,
        params_->max_linear_accel,
        params_->max_linear_decel,
        params_->max_angular_accel,
        params_->max_angular_decel,
        linear_vel, regulation_curvature, x_vel_sign, control_duration_);
    }

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
      terms.e_y, terms.e_psi, terms.sigma, terms.sigma_y, terms.sigma_psi,
      terms.kappa_pp, regulation_curvature, terms.v_gain, terms.lookahead_dist};
    ecpp_debug_pub_->publish(debug_msg);
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(
      pose, linear_vel, angular_vel, carrot_dist,
      nav2_util::geometry_utils::calculate_path_length(transformed_plan)))
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
ErrorCompensatedPurePursuitController::validateParameterUpdates(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  ControllerParameterState state{
    use_error_compensation_, params_->use_fixed_curvature_lookahead,
    ecpp_params_, ecpp_omega_max_, ecpp_error_search_window_,
    e_y_filter_.tau, ecpp_v_gain_source_, *params_,
    params_->max_linear_vel, params_->max_angular_accel};
  auto result = stageParameterUpdates(
    parameters, plugin_name_, use_legacy_omega_max_alias_, state);
  if (!result.successful) {
    return result;
  }

  result.reason = validateEcppParameters(
    state.ecpp_params, state.ecpp_omega_max, state.error_search_window,
    state.error_filter_tau, state.v_gain_source);
  if (!result.reason.empty()) {
    result.successful = false;
    return result;
  }
  result.reason = validateDynamicWindowParameters(
    state.dynamic_window_params, state.max_linear_vel, state.max_angular_accel);
  if (!result.reason.empty()) {
    result.successful = false;
  }
  return result;
}

void ErrorCompensatedPurePursuitController::updateParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  ControllerParameterState state{
    use_error_compensation_, params_->use_fixed_curvature_lookahead,
    ecpp_params_, ecpp_omega_max_, ecpp_error_search_window_,
    e_y_filter_.tau, ecpp_v_gain_source_, *params_,
    params_->max_linear_vel, params_->max_angular_accel};
  const auto result = stageParameterUpdates(
    parameters, plugin_name_, use_legacy_omega_max_alias_, state);
  if (!result.successful) {
    RCLCPP_ERROR(logger_, "Validated parameter update could not be applied: %s",
        result.reason.c_str());
    return;
  }

  const bool reset_error_filters =
    state.error_filter_tau != e_y_filter_.tau ||
    state.use_error_compensation != use_error_compensation_;
  use_error_compensation_ = state.use_error_compensation;
  ecpp_params_ = state.ecpp_params;
  ecpp_omega_max_ = state.ecpp_omega_max;
  ecpp_error_search_window_ = state.error_search_window;
  e_y_filter_.tau = state.error_filter_tau;
  e_psi_filter_.tau = state.error_filter_tau;
  if (reset_error_filters) {
    e_y_filter_.reset();
    e_psi_filter_.reset();
  }
  ecpp_v_gain_source_ = state.v_gain_source;
}

void ErrorCompensatedPurePursuitController::activate()
{
  RegulatedPurePursuitController::activate();
  ecpp_debug_pub_->on_activate();
  global_path_pub_->on_activate();
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  e_y_filter_.reset();
  e_psi_filter_.reset();
}

void ErrorCompensatedPurePursuitController::newPathReceived(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  ecpp_path_handler_->setPlan(path);
  e_y_filter_.reset();
  e_psi_filter_.reset();
}

void ErrorCompensatedPurePursuitController::deactivate()
{
  RegulatedPurePursuitController::deactivate();
  ecpp_debug_pub_->on_deactivate();
  global_path_pub_->on_deactivate();
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  last_command_velocity_ = geometry_msgs::msg::Twist();
}

void ErrorCompensatedPurePursuitController::cleanup()
{
  auto node = node_.lock();
  if (node && legacy_parameter_handler_) {
    node->remove_pre_set_parameters_callback(legacy_parameter_handler_.get());
  }
  legacy_parameter_handler_.reset();
  if (node && parameter_validation_handler_) {
    node->remove_on_set_parameters_callback(parameter_validation_handler_.get());
  }
  if (node && parameter_update_handler_) {
    node->remove_post_set_parameters_callback(parameter_update_handler_.get());
  }
  parameter_validation_handler_.reset();
  parameter_update_handler_.reset();
  param_handler_->deactivate();
  RegulatedPurePursuitController::cleanup();
  global_path_pub_.reset();
  ecpp_path_handler_.reset();
  ecpp_debug_pub_.reset();
}

void ErrorCompensatedPurePursuitController::reset()
{
  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  cancelling_ = false;
  finished_cancelling_ = false;
  last_command_velocity_ = geometry_msgs::msg::Twist();
  e_y_filter_.reset();
  e_psi_filter_.reset();
}

}  // namespace nav2_error_compensated_pure_pursuit_controller

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(
  nav2_error_compensated_pure_pursuit_controller::ErrorCompensatedPurePursuitController,
  nav2_core::Controller)
