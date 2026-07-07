// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "nav2_error_compensated_pure_pursuit_controller/ecpp_math.hpp"

namespace nav2_error_compensated_pure_pursuit_controller
{

/**
 * @class ErrorCompensatedPurePursuitController
 * @brief Pure pursuit with gated difference-gain error compensation (ECPP).
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
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  ecpp_math::EcppParams ecpp_params_;
  double ecpp_omega_max_{2.0};
  double ecpp_error_search_window_{2.0};
  std::string ecpp_v_gain_source_{"commanded"};  // "commanded" | "measured"
  ecpp_math::FirstOrderFilter e_y_filter_;
  ecpp_math::FirstOrderAngleFilter e_psi_filter_;
  rclcpp::Clock::SharedPtr clock_;

  geometry_msgs::msg::Twist last_command_velocity_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
  ecpp_debug_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ecpp_dyn_params_handler_;
};

}  // namespace nav2_error_compensated_pure_pursuit_controller

#endif  // NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER_HPP_
