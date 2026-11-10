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

#pragma once

#include <memory>
#include <limits>
#include <cstdint>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_dynamic_window_pure_pursuit_controller/dynamic_window_pure_pursuit_controller.hpp"

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "nav2_error_compensated_pure_pursuit_controller/ecpp_math.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/path_handler.hpp"

namespace nav2_error_compensated_pure_pursuit_controller
{

/**
 * @class ErrorCompensatedPurePursuitController
 * @brief Pure pursuit with gated ECPP curvature compensation.
 *
 * The curvature command is
 *   kappa = kappa_pp - sigma(e_y) * (dK_y * e_y + dK_psi * sin(e_psi))
 * where kappa_pp is the pure pursuit curvature to the lookahead point,
 * (e_y, e_psi) are path-frame tracking errors, dK_* are the differences
 * between target gains (from omega_n, zeta) and pure pursuit's intrinsic
 * gains (2/L_d^2, 2/L_d), and sigma is a sigmoid gate driven by the lateral
 * relative linearization error (e_y / L_d)^2.
 */
class ErrorCompensatedPurePursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  ErrorCompensatedPurePursuitController() = default;
  ~ErrorCompensatedPurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void reset() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  rcl_interfaces::msg::SetParametersResult validateParameterUpdates(
    const std::vector<rclcpp::Parameter> & parameters);

  void updateParameters(const std::vector<rclcpp::Parameter> & parameters);

  using WindowBounds = nav2_dynamic_window_pure_pursuit_controller::
    DynamicWindowPurePursuitController::DynamicWindowBounds;
  struct ControlState
  {
    geometry_msgs::msg::Twist previous;
    bool reached_xy{false};
    bool finished_cancelling{false};
  };
  struct ControlStep
  {
    geometry_msgs::msg::Twist command;
    WindowBounds bounds;
    ControlState state;
    geometry_msgs::msg::PoseStamped carrot, curvature_carrot;
    ecpp_math::EcppTerms terms;
    bool rotating{false};
  };
  struct PredictionContext
  {
    geometry_msgs::msg::PoseStamped start;
    nav_msgs::msg::Path plan;
    double radius{0.0}, horizon{0.0};
  };
  struct PredictionResult
  {
    nav_msgs::msg::Path path;
    std::vector<geometry_msgs::msg::PoseStamped> control_poses;
    std::vector<geometry_msgs::msg::Twist> commands;
    geometry_msgs::msg::Twist first_command;
    std::string event{"clear"};
    double distance{0.0}, event_distance{0.0}, first_scale{1.0};
    size_t steps{0};
  };
  ControlStep computeControlStep(
    const PathHandler::TransformedPlan & transformed,
    const geometry_msgs::msg::Twist & speed, double pose_cost, ControlState state);
  PredictionContext makePredictionContext(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & feedback, const geometry_msgs::msg::Twist & measured);
  PathHandler::TransformedPlan predictionLocalPlan(
    nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & pose) const;
  PredictionResult predictTracking(
    const PredictionContext & context, const ControlStep & first,
    double stop_budget = std::numeric_limits<double>::infinity());
  geometry_msgs::msg::Twist limitPredictedCollision(
    const PredictionContext & context, const ControlStep & first);
  std::string predictionPoseStatus(const geometry_msgs::msg::PoseStamped & pose);
  double stoppingTime(const geometry_msgs::msg::Twist & command) const;
  static double sweptSpeed(const geometry_msgs::msg::Twist & command, double radius);
  double stoppingDistance(const geometry_msgs::msg::Twist & command, double radius) const;
  double brakingScale(
    const geometry_msgs::msg::Twist & command, double radius, double available) const;
  double predictionHorizon(
    const geometry_msgs::msg::Twist & feedback, const geometry_msgs::msg::Twist & measured) const;
  void publishPredictionDiagnostics(
    const PredictionContext & context, const ControlStep & first,
    const PredictionResult & nominal, const PredictionResult & checked,
    const std::string & status, double elapsed_ms);
  std::string collision_prediction_mode_{"constant_velocity"};
  double collision_prediction_time_buffer_{2.0}, collision_stop_margin_{0.3};
  int64_t collision_prediction_max_steps_{300};
  bool publish_collision_prediction_{false};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    nominal_prediction_pub_, checked_prediction_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    prediction_diagnostics_pub_;

  // Configuration-only window limits. The upstream Jazzy DWPP helper is reused.
  nav2_dynamic_window_pure_pursuit_controller::DynamicWindowPurePursuitController window_;
  bool use_dynamic_window_{false};
  bool supervised_paths_{false};
  double window_v_{0.5}, window_w_{1.0}, window_a_{0.5}, window_alpha_{1.5};
  geometry_msgs::msg::Twist last_command_;
  bool command_initialized_{false};
  int64_t pending_stamp_ns_{0}, adopted_stamp_ns_{0};
  double pending_omega_n_{1.0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr adoption_pub_;
  bool ecpp_publish_debug_{false};
  ecpp_math::EcppParams ecpp_params_;
  std::string ecpp_v_gain_source_{"commanded"};  // "commanded" | "measured" | "nominal"
  std::unique_ptr<PathHandler> ecpp_path_handler_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
  ecpp_debug_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_validation_handler_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    parameter_update_handler_;
};

}  // namespace nav2_error_compensated_pure_pursuit_controller
