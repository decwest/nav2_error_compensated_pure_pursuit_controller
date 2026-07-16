// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_core/goal_checker.hpp"
#include \
  "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ecpp_controller = nav2_error_compensated_pure_pursuit_controller;

namespace
{

class RclcppFixture
{
public:
  RclcppFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppFixture()
  {
    rclcpp::shutdown();
  }
};

RclcppFixture g_rclcpp_fixture;

class TestGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {}

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    return false;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    pose_tolerance.position.x = 0.05;
    vel_tolerance = geometry_msgs::msg::Twist();
    return true;
  }
};

class TestController : public ecpp_controller::ErrorCompensatedPurePursuitController
{
public:
  bool usesErrorCompensation() const {return use_error_compensation_;}

  const ecpp_controller::ecpp_math::EcppParams & ecppParams() const {return ecpp_params_;}

  double ecppOmegaMax() const {return ecpp_omega_max_;}

  double errorSearchWindow() const {return ecpp_error_search_window_;}

  const std::string & gainSource() const {return ecpp_v_gain_source_;}

  double filterTimeConstant() const {return e_y_filter_.tau;}

  const ecpp_controller::DynamicWindowParameters & dynamicWindowParams() const
  {
    return dynamic_window_params_;
  }

  double maxLinearVelocity() const {return params_->desired_linear_vel;}

  double maxAngularAcceleration() const {return params_->max_angular_accel;}
};

struct ControllerHarness
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<TestController> controller;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr rejecting_callback;
};

ControllerHarness makeController(
  const std::string & suffix,
  const bool use_error_compensation,
  const bool use_dynamic_window,
  const bool reject_parameter_updates = false,
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "ecpp_test_" + suffix, options);
  const std::string plugin_name = "FollowPath";

  node->declare_parameter("controller_frequency", 20.0);
  node->declare_parameter(plugin_name + ".use_rotate_to_heading", false);
  node->declare_parameter(plugin_name + ".use_collision_detection", false);
  node->declare_parameter(plugin_name + ".use_regulated_linear_velocity_scaling", false);
  node->declare_parameter(plugin_name + ".use_cost_regulated_linear_velocity_scaling", false);
  node->declare_parameter(plugin_name + ".ecpp.use_error_compensation", use_error_compensation);
  node->declare_parameter(plugin_name + ".ecpp.gate_mode", "always_on");
  node->declare_parameter(plugin_name + ".use_dynamic_window", use_dynamic_window);

  auto rejection_enabled = std::make_shared<bool>(false);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr rejecting_callback;
  if (reject_parameter_updates) {
    rejecting_callback = node->add_on_set_parameters_callback(
      [rejection_enabled](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = !*rejection_enabled;
        if (!result.successful) {
          result.reason = "Rejected by the test callback";
        }
        return result;
      });
    // Keep declaration-time updates enabled. The flag is set after configure below so this
    // callback remains earlier than both controller callbacks in the LIFO on-set chain.
  }

  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap_" + suffix);
  costmap->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_shared<TestController>();
  controller->configure(node, plugin_name, transform_buffer, costmap);
  *rejection_enabled = reject_parameter_updates;
  return {node, costmap, controller, rejecting_callback};
}

nav_msgs::msg::Path makeOffsetPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = stamp;
  for (int index = -1; index <= 4; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = 0.5 * index;
    pose.pose.position.y = -0.2;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::TwistStamped computeFirstCommand(ControllerHarness & harness)
{
  harness.controller->activate();
  const auto stamp = harness.node->get_clock()->now();
  harness.controller->setPlan(makeOffsetPath(stamp));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;
  const geometry_msgs::msg::Twist current_speed;
  TestGoalChecker goal_checker;

  const auto command = harness.controller->computeVelocityCommands(
    robot_pose, current_speed, &goal_checker);
  harness.controller->deactivate();
  harness.controller->cleanup();
  harness.costmap->on_cleanup(rclcpp_lifecycle::State());
  return command;
}

}  // namespace

TEST(ControllerOptions, FourMethodCombinationsSelectIndependentLayers)
{
  auto pp_harness = makeController("pp", false, false);
  auto ecpp_harness = makeController("ecpp", true, false);
  auto dwpp_harness = makeController("dwpp", false, true);
  auto combined_harness = makeController("combined", true, true);

  const auto pp_command = computeFirstCommand(pp_harness);
  const auto ecpp_command = computeFirstCommand(ecpp_harness);
  const auto dwpp_command = computeFirstCommand(dwpp_harness);
  const auto combined_command = computeFirstCommand(combined_harness);

  // The arc-length carrot is (0.6, -0.2), giving kappa_pp = -1.0.
  EXPECT_NEAR(pp_command.twist.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(pp_command.twist.angular.z, -0.5, 1e-9);

  // ECPP changes curvature without changing the non-DWPP linear command.
  EXPECT_NEAR(ecpp_command.twist.linear.x, pp_command.twist.linear.x, 1e-9);
  EXPECT_NE(ecpp_command.twist.angular.z, pp_command.twist.angular.z);

  // From rest at 20 Hz, the official 2.5 m/s^2 dynamic window limits v to 0.125 m/s.
  EXPECT_NEAR(dwpp_command.twist.linear.x, 0.125, 1e-9);
  EXPECT_NEAR(dwpp_command.twist.angular.z, -0.125, 1e-9);
  EXPECT_NEAR(combined_command.twist.linear.x, 0.125, 1e-9);

  // With both options enabled, DWPP receives the compensated curvature rather than kappa_pp.
  EXPECT_NE(combined_command.twist.angular.z, dwpp_command.twist.angular.z);
  EXPECT_LT(
    std::fabs(combined_command.twist.angular.z),
    std::fabs(dwpp_command.twist.angular.z));
}

TEST(ControllerOptions, NonDynamicWindowEcppDoesNotClipRawAngularCommand)
{
  auto harness = makeController("raw_no_clip", true, false);
  const auto result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter("FollowPath.ecpp.omega_n", 10.0)});
  ASSERT_TRUE(result.successful) << result.reason;

  const auto command = computeFirstCommand(harness);
  // The deprecated ecpp.omega_max default is 2 rad/s. A non-DW command must remain the raw
  // v*kappa request so the downstream velocity smoother is the sole actuator constraint.
  EXPECT_GT(std::fabs(command.twist.angular.z), 2.0);
}

TEST(ControllerOptions, RejectsEcppWithFixedCurvatureLookahead)
{
  auto harness = makeController("fixed_dynamic", true, false);
  const std::string prefix = "FollowPath.";

  const auto invalid_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "use_fixed_curvature_lookahead", true)});
  EXPECT_FALSE(invalid_result.successful);
  EXPECT_FALSE(harness.node->get_parameter(
      prefix + "use_fixed_curvature_lookahead").as_bool());

  const auto switch_result = harness.node->set_parameters_atomically(
  {
    rclcpp::Parameter(prefix + "ecpp.use_error_compensation", false),
    rclcpp::Parameter(prefix + "use_fixed_curvature_lookahead", true),
  });
  ASSERT_TRUE(switch_result.successful) << switch_result.reason;
  EXPECT_FALSE(harness.controller->usesErrorCompensation());
  EXPECT_TRUE(harness.node->get_parameter(
      prefix + "use_fixed_curvature_lookahead").as_bool());

  const auto invalid_reenable_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "ecpp.use_error_compensation", true)});
  EXPECT_FALSE(invalid_reenable_result.successful);

  harness.controller->cleanup();
  harness.costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(ControllerOptions, RejectsInvalidFixedLookaheadCombinationAtStartup)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("FollowPath.use_fixed_curvature_lookahead", true),
  });
  EXPECT_THROW(
    makeController("fixed_startup", true, false, false, options),
    nav2_core::ControllerException);
}

TEST(ControllerParameters, AddedParametersUpdateAtomicallyAndRejectInvalidValues)
{
  auto harness = makeController("parameters", false, false);
  const std::string prefix = "FollowPath.";

  EXPECT_FALSE(harness.controller->usesErrorCompensation());
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, 1.0);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().zeta, 1.0);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().v_epsilon, 0.05);
  EXPECT_FALSE(harness.controller->dynamicWindowParams().use_dynamic_window);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().min_linear_vel, -0.5);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_angular_vel, 2.5);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().min_angular_vel, -2.5);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_linear_accel, 2.5);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_linear_decel, -2.5);
  EXPECT_DOUBLE_EQ(harness.controller->maxAngularAcceleration(), 3.2);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_angular_decel, -3.2);
  EXPECT_DOUBLE_EQ(harness.controller->maxLinearVelocity(), 0.5);

  const auto update_result = harness.node->set_parameters_atomically(
  {
    rclcpp::Parameter(prefix + "ecpp.use_error_compensation", true),
    rclcpp::Parameter(prefix + "ecpp.omega_n", 1.2),
    rclcpp::Parameter(prefix + "ecpp.zeta", 0.8),
    rclcpp::Parameter(prefix + "ecpp.v_epsilon", 0.1),
    rclcpp::Parameter(prefix + "ecpp.gate_mode", "product"),
    rclcpp::Parameter(prefix + "ecpp.gate_error_on", 0.2),
    rclcpp::Parameter(prefix + "ecpp.gate_error_off", 0.6),
    rclcpp::Parameter(prefix + "ecpp.gate_endpoint_value", 0.02),
    rclcpp::Parameter(prefix + "ecpp.omega_max", 1.9),
    rclcpp::Parameter(prefix + "ecpp.v_gain_source", "measured"),
    rclcpp::Parameter(prefix + "ecpp.error_search_window", 3.0),
    rclcpp::Parameter(prefix + "ecpp.error_filter_tau", 0.1),
    rclcpp::Parameter(prefix + "use_dynamic_window", true),
    rclcpp::Parameter(prefix + "min_linear_vel", -0.4),
    rclcpp::Parameter(prefix + "max_angular_vel", 2.1),
    rclcpp::Parameter(prefix + "min_angular_vel", -2.1),
    rclcpp::Parameter(prefix + "max_linear_accel", 1.5),
    rclcpp::Parameter(prefix + "max_linear_decel", -1.6),
    rclcpp::Parameter(prefix + "max_angular_accel", 4.0),
    rclcpp::Parameter(prefix + "max_angular_decel", -3.0),
    rclcpp::Parameter(prefix + "desired_linear_vel", 0.7),
    });

  ASSERT_TRUE(update_result.successful) << update_result.reason;
  EXPECT_TRUE(harness.controller->usesErrorCompensation());
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, 1.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().zeta, 0.8);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().v_epsilon, 0.1);
  EXPECT_EQ(
    harness.controller->ecppParams().gate_mode,
    ecpp_controller::ecpp_math::GateMode::PRODUCT);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_on, 0.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_off, 0.6);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_endpoint_value, 0.02);
  EXPECT_DOUBLE_EQ(harness.controller->ecppOmegaMax(), 1.9);
  EXPECT_EQ(harness.controller->gainSource(), "measured");
  EXPECT_DOUBLE_EQ(harness.controller->errorSearchWindow(), 3.0);
  EXPECT_DOUBLE_EQ(harness.controller->filterTimeConstant(), 0.1);
  EXPECT_TRUE(harness.controller->dynamicWindowParams().use_dynamic_window);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().min_linear_vel, -0.4);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_angular_vel, 2.1);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().min_angular_vel, -2.1);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_linear_accel, 1.5);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_linear_decel, -1.6);
  EXPECT_DOUBLE_EQ(harness.controller->maxAngularAcceleration(), 4.0);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_angular_decel, -3.0);
  EXPECT_DOUBLE_EQ(harness.controller->maxLinearVelocity(), 0.7);

  // In the normal configuration, the deprecated ECPP limit does not override the declared
  // canonical DWPP limit.
  const auto deprecated_limit_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "ecpp.omega_max", 1.8)});
  ASSERT_TRUE(deprecated_limit_result.successful) << deprecated_limit_result.reason;
  EXPECT_DOUBLE_EQ(harness.controller->ecppOmegaMax(), 1.8);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_angular_vel, 2.1);
  EXPECT_DOUBLE_EQ(harness.node->get_parameter(prefix + "max_angular_vel").as_double(), 2.1);

  const auto bad_ecpp_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "ecpp.omega_n", -1.0)});
  EXPECT_FALSE(bad_ecpp_result.successful);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, 1.2);

  const auto bad_deceleration_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "max_linear_decel", 0.1)});
  EXPECT_FALSE(bad_deceleration_result.successful);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_linear_decel, -1.6);

  const auto bad_gate_range_result = harness.node->set_parameters_atomically(
  {
    rclcpp::Parameter(prefix + "ecpp.gate_error_on", 0.8),
    rclcpp::Parameter(prefix + "ecpp.gate_error_off", 0.7),
    });
  EXPECT_FALSE(bad_gate_range_result.successful);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_on, 0.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_off, 0.6);

  const auto non_finite_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(
        prefix + "max_angular_vel", std::numeric_limits<double>::infinity())});
  EXPECT_FALSE(non_finite_result.successful);
  EXPECT_DOUBLE_EQ(harness.controller->dynamicWindowParams().max_angular_vel, 2.1);

  const std::vector<rclcpp::Parameter> invalid_parameters = {
    rclcpp::Parameter(prefix + "ecpp.gate_mode", "invalid"),
    rclcpp::Parameter(prefix + "ecpp.v_gain_source", "invalid"),
    rclcpp::Parameter(prefix + "ecpp.gate_endpoint_value", 0.5),
    rclcpp::Parameter(prefix + "ecpp.error_search_window", 0.0),
    rclcpp::Parameter(prefix + "ecpp.error_filter_tau", -0.1),
    rclcpp::Parameter(prefix + "min_linear_vel", 0.8),
    rclcpp::Parameter(prefix + "min_angular_vel", 2.2),
    rclcpp::Parameter(prefix + "max_linear_accel", -0.1),
    rclcpp::Parameter(prefix + "max_linear_decel", 0.1),
    rclcpp::Parameter(prefix + "max_angular_accel", -0.1),
    rclcpp::Parameter(prefix + "max_angular_decel", 0.1),
    rclcpp::Parameter(prefix + "desired_linear_vel", -0.1),
    rclcpp::Parameter(
      prefix + "ecpp.zeta", std::numeric_limits<double>::quiet_NaN()),
  };
  for (const auto & invalid_parameter : invalid_parameters) {
    const auto invalid_result = harness.node->set_parameters_atomically({invalid_parameter});
    EXPECT_FALSE(invalid_result.successful) << invalid_parameter.get_name();
  }

  const auto wrong_type_result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "max_angular_vel", "not a double")});
  EXPECT_FALSE(wrong_type_result.successful);

  // Positive minima describe a valid one-directional window and are accepted when min <= max.
  const auto positive_minima_result = harness.node->set_parameters_atomically(
  {
    rclcpp::Parameter(prefix + "min_linear_vel", 0.2),
    rclcpp::Parameter(prefix + "min_angular_vel", 0.3),
    });
  EXPECT_TRUE(positive_minima_result.successful) << positive_minima_result.reason;

  harness.controller->cleanup();
  harness.costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(ControllerParameters, RejectedCallbackChainDoesNotMutateControllerState)
{
  auto harness = makeController("callback_rejection", false, false, true);
  const std::string parameter_name = "FollowPath.ecpp.omega_n";
  const double initial_node_value = harness.node->get_parameter(parameter_name).as_double();
  const double initial_controller_value = harness.controller->ecppParams().omega_n;

  const auto result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter(parameter_name, 1.7)});

  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(harness.node->get_parameter(parameter_name).as_double(), initial_node_value);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, initial_controller_value);

  harness.controller->cleanup();
  harness.costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(ControllerParameters, StartupLegacyAngularLimitAliasAndCanonicalPrecedence)
{
  const std::string prefix = "FollowPath.";

  rclcpp::NodeOptions legacy_options;
  legacy_options.parameter_overrides(
  {
    rclcpp::Parameter(prefix + "ecpp.omega_max", 1.7),
    rclcpp::Parameter(prefix + "use_dynamic_window", true),
  });
  auto legacy_harness = makeController("legacy_override", false, true, false, legacy_options);

  EXPECT_FALSE(legacy_harness.node->has_parameter(prefix + "max_angular_vel"));
  EXPECT_DOUBLE_EQ(legacy_harness.controller->ecppOmegaMax(), 1.7);
  EXPECT_DOUBLE_EQ(legacy_harness.controller->dynamicWindowParams().max_angular_vel, 1.7);

  const auto legacy_update_result = legacy_harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "ecpp.omega_max", 1.6)});
  ASSERT_TRUE(legacy_update_result.successful) << legacy_update_result.reason;
  EXPECT_DOUBLE_EQ(legacy_harness.controller->dynamicWindowParams().max_angular_vel, 1.6);

  legacy_harness.controller->cleanup();
  legacy_harness.costmap->on_cleanup(rclcpp_lifecycle::State());

  rclcpp::NodeOptions canonical_options;
  canonical_options.parameter_overrides(
  {
    rclcpp::Parameter(prefix + "ecpp.omega_max", 1.7),
    rclcpp::Parameter(prefix + "max_angular_vel", 2.2),
    rclcpp::Parameter(prefix + "use_dynamic_window", true),
  });
  auto canonical_harness = makeController(
    "canonical_override", false, true, false, canonical_options);

  EXPECT_TRUE(canonical_harness.node->has_parameter(prefix + "max_angular_vel"));
  EXPECT_DOUBLE_EQ(canonical_harness.controller->ecppOmegaMax(), 1.7);
  EXPECT_DOUBLE_EQ(canonical_harness.controller->dynamicWindowParams().max_angular_vel, 2.2);

  const auto deprecated_update_result = canonical_harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "ecpp.omega_max", 1.6)});
  ASSERT_TRUE(deprecated_update_result.successful) << deprecated_update_result.reason;
  EXPECT_DOUBLE_EQ(canonical_harness.controller->ecppOmegaMax(), 1.6);
  EXPECT_DOUBLE_EQ(canonical_harness.controller->dynamicWindowParams().max_angular_vel, 2.2);
  EXPECT_DOUBLE_EQ(
    canonical_harness.node->get_parameter(prefix + "max_angular_vel").as_double(), 2.2);

  const auto canonical_update_result = canonical_harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "max_angular_vel", 2.0)});
  ASSERT_TRUE(canonical_update_result.successful) << canonical_update_result.reason;
  EXPECT_DOUBLE_EQ(canonical_harness.controller->dynamicWindowParams().max_angular_vel, 2.0);

  canonical_harness.controller->cleanup();
  canonical_harness.costmap->on_cleanup(rclcpp_lifecycle::State());

  rclcpp::NodeOptions canonical_only_options;
  canonical_only_options.parameter_overrides(
  {
    rclcpp::Parameter(prefix + "max_angular_vel", 2.3),
    rclcpp::Parameter(prefix + "use_dynamic_window", true),
  });
  auto canonical_only_harness = makeController(
    "canonical_only_override", false, true, false, canonical_only_options);

  EXPECT_TRUE(canonical_only_harness.node->has_parameter(prefix + "max_angular_vel"));
  EXPECT_DOUBLE_EQ(canonical_only_harness.controller->ecppOmegaMax(), 2.0);
  EXPECT_DOUBLE_EQ(
    canonical_only_harness.controller->dynamicWindowParams().max_angular_vel, 2.3);

  const auto canonical_only_deprecated_update_result =
    canonical_only_harness.node->set_parameters_atomically(
    {rclcpp::Parameter(prefix + "ecpp.omega_max", 1.5)});
  ASSERT_TRUE(canonical_only_deprecated_update_result.successful) <<
    canonical_only_deprecated_update_result.reason;
  EXPECT_DOUBLE_EQ(
    canonical_only_harness.controller->dynamicWindowParams().max_angular_vel, 2.3);

  canonical_only_harness.controller->cleanup();
  canonical_only_harness.costmap->on_cleanup(rclcpp_lifecycle::State());
}
