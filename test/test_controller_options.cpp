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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_core/goal_checker.hpp"
#include \
  "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_map.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
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
  const ecpp_controller::ecpp_math::EcppParams & ecppParams() const {return ecpp_params_;}
  const std::string & gainSource() const {return ecpp_v_gain_source_;}
  double desiredSpeed() const {return params_->base_desired_linear_vel;}
  double limitedSpeed() const {return params_->desired_linear_vel;}
  bool publishesDebug() const {return ecpp_publish_debug_;}
  size_t debugSubscribers() const {return ecpp_debug_pub_->get_subscription_count();}
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
  const std::string & gate_mode = "ey_only",
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
  node->declare_parameter(plugin_name + ".ecpp.gate_mode", gate_mode);

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

geometry_msgs::msg::TwistStamped computeCommand(
  ControllerHarness & harness, const bool reverse_path = false)
{
  const auto stamp = harness.node->get_clock()->now();
  auto path = makeOffsetPath(stamp);
  if (reverse_path) {
    for (auto & pose : path.poses) {
      pose.pose.position.x = -pose.pose.position.x;
    }
  }
  harness.controller->setPlan(path);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;
  const geometry_msgs::msg::Twist current_speed;
  TestGoalChecker goal_checker;

  return harness.controller->computeVelocityCommands(robot_pose, current_speed, &goal_checker);
}

void cleanup(ControllerHarness & harness)
{
  harness.controller->cleanup();
  harness.costmap->on_cleanup(rclcpp_lifecycle::State());
}

geometry_msgs::msg::TwistStamped computeFirstCommand(ControllerHarness & harness)
{
  harness.controller->activate();
  const auto command = computeCommand(harness);
  harness.controller->deactivate();
  cleanup(harness);
  return command;
}

}  // namespace

TEST(ControllerOptions, ThreeGateModesSelectCurvature)
{
  auto pp = makeController("pp", "off");
  auto ecpp = makeController("ecpp", "always_on");
  auto gated = makeController("gated");
  EXPECT_EQ(gated.controller->ecppParams().gate_mode,
    ecpp_controller::ecpp_math::GateMode::EY_ONLY);
  const auto pp_command = computeFirstCommand(pp);
  const auto ecpp_command = computeFirstCommand(ecpp);
  const auto gated_command = computeFirstCommand(gated);

  // The arc-length carrot is (0.6, -0.2), giving kappa_pp = -1.0.
  EXPECT_NEAR(pp_command.twist.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(pp_command.twist.angular.z, -0.5, 1e-9);
  // At v0 = 0.5 and omega_n = 1, Ky = 4; intrinsic PP gain is 2/0.6^2.
  const double compensated = 0.5 * (-1.0 - (4.0 - 2.0 / 0.36) * 0.2);
  EXPECT_NEAR(ecpp_command.twist.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(ecpp_command.twist.angular.z, compensated, 1e-9);
  EXPECT_GT(gated_command.twist.angular.z, pp_command.twist.angular.z);
  EXPECT_LT(gated_command.twist.angular.z, ecpp_command.twist.angular.z);
}

TEST(ControllerOptions, EcppReturnsRawAngularCommand)
{
  auto harness = makeController("raw_command", "always_on");
  ASSERT_TRUE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter("FollowPath.ecpp.omega_n", 10.0)}).successful);
  const auto command = computeFirstCommand(harness);
  // Physical angular limits belong to the downstream velocity smoother.
  EXPECT_GT(std::fabs(command.twist.angular.z), 2.0);
}

TEST(ControllerOptions, PruningLookaheadAndCompensationShareProjection)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.max_robot_pose_search_dist", 0.1),
    rclcpp::Parameter("FollowPath.lookahead_dist", 0.4)});
  auto harness = makeController("shared_projection", "always_on", false, options);
  harness.controller->activate();

  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = harness.node->get_clock()->now();
  for (const auto & [x, y] : std::vector<std::pair<double, double>>{
    {-0.5, 0.3}, {0.5, 0.3}, {0.5, 0.05}, {-0.5, 0.05}})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  harness.controller->setPlan(path);
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = path.header;
  robot_pose.pose.orientation.w = 1.0;
  TestGoalChecker goal_checker;
  const auto command = harness.controller->computeVelocityCommands(
    robot_pose, geometry_msgs::msg::Twist(), &goal_checker);

  // The bound selects the first segment, despite the closer returning branch. Its carrot is
  // (0.4, 0.3), e_y = -0.3, and e_theta = 0. Re-searching locally would select the return branch.
  const double kappa_pp = 2.0 * 0.3 / (0.4 * 0.4 + 0.3 * 0.3);
  const double delta_ky = 4.0 - 2.0 / (0.4 * 0.4);
  EXPECT_NEAR(command.twist.linear.x, 0.5, 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.5 * (kappa_pp - delta_ky * (-0.3)), 1e-12);

  harness.controller->deactivate();
  cleanup(harness);
}

TEST(ControllerOptions, RejectsEcppWithFixedCurvatureLookahead)
{
  auto harness = makeController("fixed_dynamic");
  const std::string prefix = "FollowPath.";
  EXPECT_FALSE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter(prefix + "use_fixed_curvature_lookahead", true)}).successful);
  EXPECT_FALSE(harness.node->get_parameter(prefix + "use_fixed_curvature_lookahead").as_bool());
  ASSERT_TRUE(harness.node->set_parameters_atomically({
    rclcpp::Parameter(prefix + "ecpp.gate_mode", "off"),
    rclcpp::Parameter(prefix + "use_fixed_curvature_lookahead", true)}).successful);
  EXPECT_EQ(harness.controller->ecppParams().gate_mode, ecpp_controller::ecpp_math::GateMode::OFF);
  EXPECT_FALSE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter(prefix + "ecpp.gate_mode", "ey_only")}).successful);
  EXPECT_FALSE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter(prefix + "ecpp.gate_mode", "always_on")}).successful);
  ASSERT_TRUE(harness.node->set_parameters_atomically({
    rclcpp::Parameter(prefix + "ecpp.gate_mode", "ey_only"),
    rclcpp::Parameter(prefix + "use_fixed_curvature_lookahead", false)}).successful);
  cleanup(harness);
}

TEST(ControllerParameters, StartupValidationMatchesDynamicValidation)
{
  const std::vector<rclcpp::Parameter> invalid = {
    rclcpp::Parameter("FollowPath.use_fixed_curvature_lookahead", true),
    rclcpp::Parameter("FollowPath.desired_linear_vel", 0.0),
    rclcpp::Parameter("FollowPath.desired_linear_vel", std::numeric_limits<double>::infinity()),
    rclcpp::Parameter("FollowPath.ecpp.v_epsilon", -0.05),
    rclcpp::Parameter("FollowPath.max_angular_accel", -0.1),
    rclcpp::Parameter("FollowPath.ecpp.gate_error_off", 0.05),
    rclcpp::Parameter("FollowPath.ecpp.gate_mode", "invalid"),
  };
  for (size_t index = 0; index < invalid.size(); ++index) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({invalid[index]});
    EXPECT_THROW(makeController("invalid_" + std::to_string(index), "ey_only", false, options),
      nav2_core::ControllerException);
  }
}

TEST(ControllerParameters, UpdatesAreValidatedBeforeApplication)
{
  auto harness = makeController("parameters", "off");
  const std::string prefix = "FollowPath.";
  EXPECT_FALSE(harness.controller->publishesDebug());
  const auto result = harness.node->set_parameters_atomically({
    rclcpp::Parameter(prefix + "ecpp.omega_n", 1.2),
    rclcpp::Parameter(prefix + "ecpp.zeta", 0.8),
    rclcpp::Parameter(prefix + "ecpp.v_epsilon", 0.1),
    rclcpp::Parameter(prefix + "ecpp.gate_mode", "ey_only"),
    rclcpp::Parameter(prefix + "ecpp.gate_error_on", 0.2),
    rclcpp::Parameter(prefix + "ecpp.gate_error_off", 0.6),
    rclcpp::Parameter(prefix + "ecpp.gate_endpoint_value", 0.02),
    rclcpp::Parameter(prefix + "ecpp.v_gain_source", "measured"),
    rclcpp::Parameter(prefix + "ecpp.publish_debug", true),
    rclcpp::Parameter(prefix + "desired_linear_vel", 0.7)});
  ASSERT_TRUE(result.successful) << result.reason;
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, 1.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().zeta, 0.8);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().v_epsilon, 0.1);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_on, 0.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_off, 0.6);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_endpoint_value, 0.02);
  EXPECT_EQ(harness.controller->gainSource(), "measured");
  EXPECT_TRUE(harness.controller->publishesDebug());
  EXPECT_DOUBLE_EQ(harness.controller->desiredSpeed(), 0.7);

  const std::vector<rclcpp::Parameter> invalid = {
    rclcpp::Parameter(prefix + "ecpp.omega_n", -1.0),
    rclcpp::Parameter(prefix + "ecpp.gate_mode", "invalid"),
    rclcpp::Parameter(prefix + "ecpp.v_gain_source", "invalid"),
    rclcpp::Parameter(prefix + "ecpp.gate_endpoint_value", 0.5),
    rclcpp::Parameter(prefix + "ecpp.v_epsilon", 0.0),
    rclcpp::Parameter(prefix + "desired_linear_vel", -0.1),
    rclcpp::Parameter(prefix + "max_angular_accel", -0.1),
    rclcpp::Parameter(prefix + "max_angular_accel", std::numeric_limits<double>::infinity()),
    rclcpp::Parameter(prefix + "desired_linear_vel", 0.0),
    rclcpp::Parameter(prefix + "ecpp.zeta", std::numeric_limits<double>::quiet_NaN()),
    rclcpp::Parameter(prefix + "desired_linear_vel", std::numeric_limits<double>::infinity()),
    rclcpp::Parameter(prefix + "ecpp.omega_n", "not a double"),
    rclcpp::Parameter(prefix + "ecpp.publish_debug", "not a bool"),
  };
  for (const auto & parameter : invalid) {
    EXPECT_FALSE(harness.node->set_parameters_atomically({parameter}).successful)
      << parameter.get_name();
  }
  EXPECT_FALSE(harness.node->set_parameters_atomically({
    rclcpp::Parameter(prefix + "ecpp.omega_n", 1.5),
    rclcpp::Parameter(prefix + "ecpp.gate_error_on", 0.8),
    rclcpp::Parameter(prefix + "ecpp.gate_error_off", 0.7)}).successful);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, 1.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_on, 0.2);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().gate_error_off, 0.6);
  EXPECT_DOUBLE_EQ(harness.controller->desiredSpeed(), 0.7);
  cleanup(harness);
}

TEST(ControllerParameters, RejectedCallbackChainDoesNotMutateEcppState)
{
  auto harness = makeController("callback_rejection", "ey_only", true);
  EXPECT_FALSE(harness.node->set_parameters_atomically({
    rclcpp::Parameter("FollowPath.ecpp.omega_n", 1.7),
    rclcpp::Parameter("FollowPath.ecpp.publish_debug", true)}).successful);
  EXPECT_DOUBLE_EQ(harness.node->get_parameter("FollowPath.ecpp.omega_n").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(harness.controller->ecppParams().omega_n, 1.0);
  EXPECT_FALSE(harness.controller->publishesDebug());
  cleanup(harness);
}

TEST(ControllerParameters, SpeedLimitsPreserveDesignReferenceAndExplicitUpdatesChangeIt)
{
  auto harness = makeController("speed_limit", "always_on");
  harness.controller->activate();
  const double kappa_pp = -1.0;
  const double intrinsic_gain = 2.0 / 0.36;
  const auto expected_omega = [&](double speed, double configured_omega) {
      return speed * (kappa_pp - (std::pow(configured_omega / (speed + 0.05), 2) -
             intrinsic_gain) * 0.2);
    };
  harness.controller->setSpeedLimit(0.25, false);
  EXPECT_DOUBLE_EQ(harness.controller->desiredSpeed(), 0.5);
  EXPECT_NEAR(computeCommand(harness).twist.angular.z, expected_omega(0.25, 1.1), 1e-9);
  harness.controller->setSpeedLimit(40.0, true);
  EXPECT_NEAR(harness.controller->limitedSpeed(), 0.2, 1e-12);
  EXPECT_NEAR(computeCommand(harness).twist.angular.z, expected_omega(0.2, 1.1), 1e-9);
  harness.controller->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_DOUBLE_EQ(harness.controller->limitedSpeed(), 0.5);
  EXPECT_NEAR(computeCommand(harness).twist.angular.z, expected_omega(0.5, 1.1), 1e-9);
  ASSERT_TRUE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter("FollowPath.desired_linear_vel", 0.8)}).successful);
  EXPECT_DOUBLE_EQ(harness.controller->desiredSpeed(), 0.8);
  EXPECT_NEAR(computeCommand(harness).twist.angular.z, expected_omega(0.8, 1.0625), 1e-9);
  harness.controller->deactivate();
  cleanup(harness);
}

TEST(ControllerOptions, MeasuredGainSpeedIsRegularizedAtRest)
{
  auto harness = makeController("measured", "always_on");
  ASSERT_TRUE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter("FollowPath.ecpp.v_gain_source", "measured")}).successful);
  // Odometry is zero, while the requested forward speed is 0.5 m/s.
  const auto command = computeFirstCommand(harness);
  const double expected_ky = std::pow(1.1 / 0.05, 2);
  EXPECT_NEAR(command.twist.angular.z, 0.5 * (-1.0 - (expected_ky - 2.0 / 0.36) * 0.2), 1e-9);
  EXPECT_TRUE(std::isfinite(command.twist.angular.z));
}

TEST(ControllerOptions, ReversingUsesPurePursuitCurvature)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("FollowPath.allow_reversing", true)});
  for (const std::string mode : {"off", "ey_only", "always_on"}) {
    auto harness = makeController("reverse_" + mode, mode, false, options);
    harness.controller->activate();
    const auto command = computeCommand(harness, true);
    // The carrot is (-0.6, -0.2), so kappa_pp = -1 while v = -0.5.
    EXPECT_NEAR(command.twist.linear.x, -0.5, 1e-9);
    EXPECT_NEAR(command.twist.angular.z, 0.5, 1e-9);
    harness.controller->deactivate();
    cleanup(harness);
  }
}

TEST(ControllerOptions, DebugPublishingCanBeEnabledAndDisabled)
{
  auto harness = makeController("debug");
  auto subscription = harness.node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "FollowPath/ecpp_debug", 1, [](std_msgs::msg::Float64MultiArray::ConstSharedPtr) {});
  harness.controller->activate();
  for (int attempt = 0; attempt < 200 && harness.controller->debugSubscribers() == 0; ++attempt) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(harness.controller->debugSubscribers(), 0u);
  std_msgs::msg::Float64MultiArray message;
  rclcpp::MessageInfo info;
  computeCommand(harness);
  rclcpp::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(subscription->take(message, info));
  ASSERT_TRUE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter("FollowPath.ecpp.publish_debug", true)}).successful);
  bool received = false;
  computeCommand(harness);
  for (int attempt = 0; attempt < 200 && !received; ++attempt) {
    received = subscription->take(message, info);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received);
  ASSERT_EQ(message.data.size(), 9u);
  EXPECT_DOUBLE_EQ(message.data[4], 1.0);
  EXPECT_GT(message.data[2], 0.0);
  ASSERT_TRUE(harness.node->set_parameters_atomically(
      {rclcpp::Parameter("FollowPath.ecpp.publish_debug", false)}).successful);
  computeCommand(harness);
  rclcpp::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(subscription->take(message, info));
  harness.controller->deactivate();
  cleanup(harness);
}

TEST(PublicConfigurations, RosParserLoadsEveryFileAndEcppController)
{
  for (const std::string filename : {
    "example_param.yaml", "paper_experiment1_params.yaml", "paper_experiment2_params.yaml"})
  {
    const auto parameter_map = rclcpp::parameter_map_from_yaml_file(
      (std::string(TEST_CONFIG_DIR) + "/" + filename).c_str());
    const auto & parameters = parameter_map.at("/controller_server");
    size_t controller_count = 0;
    size_t ecpp_count = 0;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "controller_plugins") {
        controller_count = parameter.as_string_array().size();
      }
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        parameter.as_string() !=
        "nav2_error_compensated_pure_pursuit_controller::ErrorCompensatedPurePursuitController")
      {
        continue;
      }
      const auto prefix = parameter.get_name().substr(0, parameter.get_name().size() - 6);
      std::vector<rclcpp::Parameter> overrides;
      for (const auto & setting : parameters) {
        if (setting.get_name() == "controller_frequency") {
          overrides.push_back(setting);
        } else if (setting.get_name().rfind(prefix, 0) == 0) {
          overrides.emplace_back("FollowPath." + setting.get_name().substr(prefix.size()),
            setting.get_parameter_value());
        }
      }
      rclcpp::NodeOptions options;
      options.parameter_overrides(overrides);
      auto harness = makeController("yaml_" + std::to_string(ecpp_count), "ey_only", false,
        options);
      EXPECT_DOUBLE_EQ(harness.controller->desiredSpeed(), 0.5);
      cleanup(harness);
      ++ecpp_count;
    }
    const size_t expected_count = filename == "example_param.yaml" ? 1u :
      (filename == "paper_experiment1_params.yaml" ? 7u : 5u);
    EXPECT_EQ(controller_count, expected_count) << filename;
    EXPECT_EQ(ecpp_count, filename == "paper_experiment2_params.yaml" ? 3u : expected_count);
  }
}
