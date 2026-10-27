// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include \
  "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

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
    const nav2::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {}

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &,
    const nav_msgs::msg::Path &) override
  {
    return false;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance,
    double & path_length_tolerance) override
  {
    pose_tolerance.position.x = 0.05;
    vel_tolerance = geometry_msgs::msg::Twist();
    path_length_tolerance = 0.05;
    return true;
  }

  bool isGoalXYReached(
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist &,
    const nav_msgs::msg::Path &) override
  {
    return std::hypot(pose.position.x - goal.position.x,
             pose.position.y - goal.position.y) <= 0.05;
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

  const nav2_regulated_pure_pursuit_controller::Parameters & dynamicWindowParams() const
  {
    return *params_;
  }

  double maxLinearVelocity() const {return params_->max_linear_vel;}

  double maxAngularAcceleration() const {return params_->max_angular_accel;}

  void setRobotPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.rotation = pose.pose.orientation;
    tf_->setTransform(transform, "ecpp_tracking_test");
  }
};

struct ControllerHarness
{
  nav2::LifecycleNode::SharedPtr node;
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
  auto node = std::make_shared<nav2::LifecycleNode>(
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
  // These tests inject transforms synchronously before each lookup.
  transform_buffer->setUsingDedicatedThread(true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap_" + suffix);
  costmap->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_shared<TestController>();
  controller->configure(node, plugin_name, transform_buffer, costmap);
  controller->activate();
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

geometry_msgs::msg::Twist commandAtOrigin(
  ControllerHarness & harness, const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & speed = geometry_msgs::msg::Twist())
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  pose.pose.orientation.w = 1.0;
  TestGoalChecker checker;
  return harness.controller->computeVelocityCommands(
    pose, speed, &checker, path, path.poses.back()).twist;
}

void finish(ControllerHarness & harness)
{
  harness.controller->deactivate();
  harness.controller->cleanup();
  harness.costmap->on_cleanup(rclcpp_lifecycle::State());
}

geometry_msgs::msg::TwistStamped computeFirstCommand(ControllerHarness & harness)
{
  const auto stamp = harness.node->get_clock()->now();
  const auto path = makeOffsetPath(stamp);
  harness.controller->newPathReceived(path);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;
  const geometry_msgs::msg::Twist current_speed;
  TestGoalChecker goal_checker;

  const auto command = harness.controller->computeVelocityCommands(
    robot_pose, current_speed, &goal_checker, path, path.poses.back());
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
    rclcpp::Parameter(prefix + "max_linear_vel", 0.7),
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
    rclcpp::Parameter(prefix + "max_linear_vel", -0.1),
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

  EXPECT_TRUE(legacy_harness.node->has_parameter(prefix + "max_angular_vel"));
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

TEST(ControllerOptions, UpstreamDwppReceivesCompensatedCurvatureAndRegulatedGainSpeed)
{
  for (const bool compensation : {false, true}) {
    auto raw = makeController("reference" + std::to_string(compensation), compensation, false);
    auto dw = makeController("window" + std::to_string(compensation), compensation, true);
    const auto path = makeOffsetPath(raw.node->get_clock()->now());
    raw.controller->newPathReceived(path);
    dw.controller->newPathReceived(path);
    const auto reference = commandAtOrigin(raw, path);
    const double curvature = reference.angular.z / reference.linear.x;
    geometry_msgs::msg::Twist previous;
    const auto & p = dw.controller->dynamicWindowParams();
    for (int cycle = 0; cycle < 12; ++cycle) {
      // Deliberately different odometry: DWPP uses the previous command, and the default
      // ECPP gain uses the pre-window regulated speed, not the window-limited speed.
      geometry_msgs::msg::Twist measured;
      measured.linear.x = 0.07;
      const auto command = commandAtOrigin(dw, path, measured);
      const auto expected = nav2_regulated_pure_pursuit_controller::
        dynamic_window_pure_pursuit::computeDynamicWindowVelocities(
        previous, p.max_linear_vel, p.min_linear_vel, p.max_angular_vel, p.min_angular_vel,
        p.max_linear_accel, p.max_linear_decel, p.max_angular_accel, p.max_angular_decel,
        reference.linear.x, curvature, 1.0, 0.05);
      EXPECT_NEAR(command.linear.x, std::get<0>(expected), 1e-10);
      EXPECT_NEAR(command.angular.z, std::get<1>(expected), 1e-10);
      EXPECT_LE(command.linear.x, p.max_linear_vel + 1e-10);
      EXPECT_GE(command.linear.x, p.min_linear_vel - 1e-10);
      EXPECT_LE(command.angular.z, p.max_angular_vel + 1e-10);
      EXPECT_GE(command.angular.z, p.min_angular_vel - 1e-10);
      EXPECT_LE(command.linear.x - previous.linear.x, p.max_linear_accel * 0.05 + 1e-10);
      EXPECT_GE(command.linear.x - previous.linear.x, p.max_linear_decel * 0.05 - 1e-10);
      EXPECT_LE(command.angular.z - previous.angular.z, p.max_angular_accel * 0.05 + 1e-10);
      EXPECT_GE(command.angular.z - previous.angular.z, p.max_angular_decel * 0.05 - 1e-10);
      previous = command;
    }
    finish(raw);
    finish(dw);
  }
}

TEST(ControllerOptions, ReplanningPreservesWindowAndResetRestartsFromRest)
{
  auto harness = makeController("replan_reset", true, true);
  auto path = makeOffsetPath(harness.node->get_clock()->now());
  harness.controller->newPathReceived(path);
  geometry_msgs::msg::Twist previous;
  for (int cycle = 0; cycle < 8; ++cycle) {
    previous = commandAtOrigin(harness, path);
  }
  for (auto & pose : path.poses) {
    pose.pose.position.y *= -1.0;
  }
  harness.controller->newPathReceived(path);
  const auto replanned = commandAtOrigin(harness, path);
  EXPECT_LE(std::fabs(replanned.angular.z - previous.angular.z), 3.2 * 0.05 + 1e-10);
  EXPECT_LE(std::fabs(replanned.linear.x - previous.linear.x), 2.5 * 0.05 + 1e-10);

  harness.controller->reset();
  const auto restarted = commandAtOrigin(harness, path);
  EXPECT_NEAR(restarted.linear.x, 0.125, 1e-10);
  EXPECT_LE(std::fabs(restarted.angular.z), 3.2 * 0.05 + 1e-10);
  harness.controller->deactivate();
  harness.controller->activate();
  const auto reactivated = commandAtOrigin(harness, path);
  EXPECT_NEAR(reactivated.linear.x, restarted.linear.x, 1e-10);
  EXPECT_NEAR(reactivated.angular.z, restarted.angular.z, 1e-10);
  finish(harness);
}

TEST(ControllerOptions, ReverseDisablesCompensationWithAndWithoutDwpp)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("FollowPath.allow_reversing", true)});
  for (const bool window : {false, true}) {
    auto pp = makeController("reverse_pp" + std::to_string(window), false, window, false, options);
    auto ecpp = makeController("reverse_ecpp" + std::to_string(window), true, window, false, options);
    auto path = makeOffsetPath(pp.node->get_clock()->now());
    for (auto & pose : path.poses) {
      pose.pose.position.x *= -1.0;
    }
    pp.controller->newPathReceived(path);
    ecpp.controller->newPathReceived(path);
    for (int cycle = 0; cycle < 5; ++cycle) {
      const auto plain = commandAtOrigin(pp, path);
      const auto compensated = commandAtOrigin(ecpp, path);
      EXPECT_LT(plain.linear.x, 0.0);
      EXPECT_NEAR(compensated.linear.x, plain.linear.x, 1e-10);
      EXPECT_NEAR(compensated.angular.z, plain.angular.z, 1e-10);
    }
    finish(pp);
    finish(ecpp);
  }
}

TEST(ControllerOptions, EndpointAndMeasuredZeroSpeedRemainFiniteInAllFourModes)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("FollowPath.ecpp.v_gain_source", "measured")});
  for (const bool compensation : {false, true}) {
    for (const bool window : {false, true}) {
      auto harness = makeController(
        "endpoint" + std::to_string(compensation) + std::to_string(window),
        compensation, window, false, options);
      auto path = makeOffsetPath(harness.node->get_clock()->now());
      path.poses.resize(2);
      path.poses[0].pose.position.x = -0.05;
      path.poses[1].pose.position.x = 0.05;
      for (auto & pose : path.poses) {
        pose.pose.position.y = 0.0;
      }
      harness.controller->newPathReceived(path);
      const auto command = commandAtOrigin(harness, path);
      EXPECT_TRUE(std::isfinite(command.linear.x));
      EXPECT_TRUE(std::isfinite(command.angular.z));
      EXPECT_LE(command.linear.x, 0.125);
      EXPECT_DOUBLE_EQ(command.angular.z, 0.0);

      // A single endpoint is accepted and an empty processed plan is rejected cleanly.
      path.poses.erase(path.poses.begin());
      harness.controller->newPathReceived(path);
      EXPECT_TRUE(std::isfinite(commandAtOrigin(harness, path).linear.x));
      geometry_msgs::msg::PoseStamped pose = path.poses.front();
      TestGoalChecker checker;
      EXPECT_THROW(harness.controller->computeVelocityCommands(
          pose, geometry_msgs::msg::Twist(), &checker, nav_msgs::msg::Path(), pose),
        nav2_core::InvalidPath);
      finish(harness);
    }
  }
}

TEST(ControllerOptions, BaseSpeedLimitAndFinalCollisionCheckAreRetained)
{
  for (const bool compensation : {false, true}) {
    for (const bool window : {false, true}) {
      auto harness = makeController(
        "limits" + std::to_string(compensation) + std::to_string(window),
        compensation, window);
      const auto path = makeOffsetPath(harness.node->get_clock()->now());
      harness.controller->newPathReceived(path);
      harness.controller->setSpeedLimit(0.2, false);
      EXPECT_LE(commandAtOrigin(harness, path).linear.x, 0.2);
      harness.controller->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
      EXPECT_DOUBLE_EQ(harness.controller->maxLinearVelocity(), 0.5);

      const auto result = harness.node->set_parameters_atomically(
        {rclcpp::Parameter("FollowPath.use_collision_detection", true)});
      ASSERT_TRUE(result.successful) << result.reason;
      auto * costmap = harness.costmap->getCostmap();
      costmap->resizeMap(100, 100, 0.1, -5.0, -5.0);
      // Occupy the robot's entire footprint, including its perimeter.
      for (unsigned int x = 40; x <= 60; ++x) {
        for (unsigned int y = 40; y <= 60; ++y) {
          costmap->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
      EXPECT_THROW(commandAtOrigin(harness, path), nav2_core::NoValidControl);
      finish(harness);
    }
  }
}

TEST(ControllerOptions, ReportsTrackingResponseSeparatelyForEachLayerCombination)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.max_linear_accel", 0.5),
    rclcpp::Parameter("FollowPath.max_linear_decel", -0.5),
    rclcpp::Parameter("FollowPath.max_angular_accel", 1.0),
    rclcpp::Parameter("FollowPath.max_angular_decel", -1.0),
    rclcpp::Parameter("FollowPath.max_angular_vel", 1.0),
    rclcpp::Parameter("FollowPath.min_angular_vel", -1.0),
  });
  for (const bool compensation : {false, true}) {
    for (const bool window : {false, true}) {
      const auto label = "ecpp" + std::to_string(compensation) + "_dwpp" + std::to_string(window);
      auto harness = makeController(label, compensation, window, false, options);
      harness.costmap->getCostmap()->resizeMap(400, 100, 0.1, -2.0, -5.0);
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.header.stamp = harness.node->now();
      for (int index = -10; index <= 300; ++index) {
        geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header = path.header;
        waypoint.pose.position.x = 0.1 * index;
        waypoint.pose.orientation.w = 1.0;
        path.poses.push_back(waypoint);
      }
      harness.controller->newPathReceived(path);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.y = 0.2;
      double yaw = 0.15;
      double integrated_error = 0.0;
      double peak_error = 0.0;
      geometry_msgs::msg::Twist speed;
      TestGoalChecker checker;
      constexpr double dt = 0.05;
      for (int cycle = 0; cycle < 240; ++cycle) {
        pose.header.stamp = harness.node->now();
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        harness.controller->setRobotPose(pose);
        const auto command = harness.controller->computeVelocityCommands(
          pose, speed, &checker, path, path.poses.back()).twist;
        ASSERT_TRUE(std::isfinite(command.linear.x)) << label;
        ASSERT_TRUE(std::isfinite(command.angular.z)) << label;
        // A deterministic unicycle plant; these results are not a physical robot benchmark.
        pose.pose.position.x += command.linear.x * std::cos(yaw) * dt;
        pose.pose.position.y += command.linear.x * std::sin(yaw) * dt;
        yaw += command.angular.z * dt;
        integrated_error += std::fabs(pose.pose.position.y) * dt;
        peak_error = std::max(peak_error, std::fabs(pose.pose.position.y));
        speed = command;
      }
      RecordProperty(label + "_final_lateral_error_m", std::to_string(pose.pose.position.y));
      RecordProperty(label + "_integrated_absolute_error_ms", std::to_string(integrated_error));
      RecordProperty(label + "_peak_lateral_error_m", std::to_string(peak_error));
      EXPECT_LT(std::fabs(pose.pose.position.y), 0.05) << label;
      finish(harness);
    }
  }
}

TEST(ControllerOptions, DeceleratingCancelStopsAndResetAllowsRestart)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.use_cancel_deceleration", true),
    rclcpp::Parameter("FollowPath.cancel_deceleration", 1.0),
  });
  for (const bool compensation : {false, true}) {
    for (const bool window : {false, true}) {
      auto harness = makeController(
        "cancel" + std::to_string(compensation) + std::to_string(window),
        compensation, window, false, options);
      const auto path = makeOffsetPath(harness.node->get_clock()->now());
      harness.controller->newPathReceived(path);
      geometry_msgs::msg::Twist speed;
      for (int cycle = 0; cycle < 8; ++cycle) {
        speed = commandAtOrigin(harness, path, speed);
      }
      ASSERT_GT(speed.linear.x, 0.0);
      EXPECT_FALSE(harness.controller->cancel());
      bool stopped = false;
      for (int cycle = 0; cycle < 30 && !stopped; ++cycle) {
        const auto command = commandAtOrigin(harness, path, speed);
        EXPECT_LE(command.linear.x, speed.linear.x + 1e-9);
        EXPECT_GE(command.linear.x, -1e-9);
        speed = command;
        stopped = harness.controller->cancel();
      }
      EXPECT_TRUE(stopped);
      EXPECT_NEAR(speed.linear.x, 0.0, 1e-9);
      harness.controller->reset();
      const auto restarted = commandAtOrigin(harness, path);
      EXPECT_GT(restarted.linear.x, 0.0);
      finish(harness);
    }
  }
}
