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

#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include "gtest/gtest.h"
#include \
  "nav2_error_compensated_pure_pursuit_controller/error_compensated_pure_pursuit_controller.hpp"
#include "tf2/utils.hpp"

using EcppController =
  nav2_error_compensated_pure_pursuit_controller::ErrorCompensatedPurePursuitController;
class PredictionController : public EcppController
{
public:
  using EcppController::makePredictionContext;
  using EcppController::predictTracking;
  using EcppController::predictionHorizon;
  using EcppController::stoppingDistance;
  using EcppController::brakingScale;
  using EcppController::limitPredictedCollision;
  void maxSteps(int value) {collision_prediction_max_steps_ = value;}
  void enable(bool value) {params_->use_collision_detection = value;}
  void seed(const geometry_msgs::msg::Twist & speed)
  {
    last_command_ = speed; command_initialized_ = true;
  }
  auto command() const {return last_command_;}
  auto plan() const {return ecpp_path_handler_->getPlan();}
  auto flags() const
  {
    return std::make_tuple(command_initialized_, has_reached_xy_tolerance_,
      finished_cancelling_, is_rotating_to_heading_, adopted_stamp_ns_, pending_stamp_ns_);
  }
  bool constantCollision(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & command, double carrot = 1.0)
  {
    return collision_checker_->isCollisionImminent(pose, command.linear.x, command.angular.z,
      carrot);
  }
  ControlStep firstStep(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed)
  {
    const auto transformed = ecpp_path_handler_->transformGlobalPlan(
      pose, params_->max_robot_pose_search_dist);
    return computeControlStep(transformed, speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y),
             {command_initialized_ ? last_command_ : speed,
               has_reached_xy_tolerance_, finished_cancelling_});
  }
};
class GoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & speed) override
  {
    pose.position.x = 0.05; speed = geometry_msgs::msg::Twist(); return true;
  }
};
inline geometry_msgs::msg::Twist velocity(double x, double y, double w)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = x;
  result.linear.y = y;
  result.angular.z = w;
  return result;
}

inline geometry_msgs::msg::PoseStamped worldPose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "odom";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(yaw / 2.0);
  result.pose.orientation.w = std::cos(yaw / 2.0);
  return result;
}

struct PredictionHarness
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::shared_ptr<tf2_ros::Buffer> tf;
  PredictionController controller;
  geometry_msgs::msg::PoseStamped pose = worldPose(0, 0, 0);

  explicit PredictionHarness(double stop_margin = 0.3)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("FollowPath.collision_prediction_mode", "closed_loop"),
      rclcpp::Parameter("FollowPath.collision_stop_margin", stop_margin),
      rclcpp::Parameter("FollowPath.use_dynamic_window", true),
      rclcpp::Parameter("FollowPath.publish_collision_prediction", true)});
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("prediction_test", options);
    node->declare_parameter("controller_frequency", 30.0);
    node->declare_parameter("FollowPath.use_collision_detection", false);
    node->declare_parameter("FollowPath.use_rotate_to_heading", false);
    node->declare_parameter("FollowPath.use_regulated_linear_velocity_scaling", false);
    node->declare_parameter("FollowPath.use_cost_regulated_linear_velocity_scaling", false);
    node->declare_parameter("FollowPath.max_allowed_time_to_collision_up_to_carrot", 2.0);
    node->declare_parameter("FollowPath.approach_velocity_scaling_dist", 0.6);
    node->declare_parameter("FollowPath.min_approach_linear_velocity", 0.03);
    node->declare_parameter("FollowPath.max_robot_pose_search_dist", 2.5);
    node->declare_parameter("FollowPath.desired_linear_vel", 0.5);
    node->declare_parameter("FollowPath.lookahead_dist", 1.0);
    node->declare_parameter("FollowPath.ecpp.v_gain_source", "nominal");
    node->declare_parameter("FollowPath.ecpp.omega_n", 1.5);
    node->declare_parameter("FollowPath.ecpp.gate_mode", "always_on");
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("prediction_map");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    // Below we supply the already padded Castella footprint.
    map->set_parameter(rclcpp::Parameter("footprint_padding", 0.0));
    map->on_configure(rclcpp_lifecycle::State());
    map->getCostmap()->resizeMap(200, 200, 0.05, -5.0, -5.0);
    clear();
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = footprint[1].x = 0.42;
    footprint[2].x = footprint[3].x = -0.48;
    footprint[0].y = footprint[3].y = 0.31;
    footprint[1].y = footprint[2].y = -0.31;
    map->setRobotFootprint(footprint);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    updateTransform();
    controller.configure(node, "FollowPath", tf, map);
    controller.activate();
  }
  ~PredictionHarness()
  {
    controller.deactivate();
    controller.cleanup();
    map->on_cleanup(rclcpp_lifecycle::State());
  }
  void clear()
  {
    auto * grid = map->getCostmap();
    std::fill(grid->getCharMap(), grid->getCharMap() + grid->getSizeInCellsX() *
      grid->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
  }
  void updateTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.rotation = pose.pose.orientation;
    tf->setTransform(transform, "prediction_test", true);
  }
  void obstacle(double x, double y)
  {
    unsigned int mx, my;
    if (!map->getCostmap()->worldToMap(x, y, mx, my)) {
      throw std::runtime_error("obstacle outside map");
    }
    map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  void advance(const geometry_msgs::msg::Twist & command)
  {
    const auto yaw = tf2::getYaw(pose.pose.orientation);
    geometry_msgs::msg::Pose2D start;
    start.x = pose.pose.position.x; start.y = pose.pose.position.y; start.theta = yaw;
    const double dt = 1.0 / 30;
    const double angle = command.angular.z * dt;
    const double dx = std::abs(command.angular.z) > 1e-10 ?
      command.linear.x / command.angular.z * (std::sin(yaw + angle) - std::sin(yaw)) :
      command.linear.x * dt * std::cos(yaw);
    const double dy = std::abs(command.angular.z) > 1e-10 ?
      command.linear.x / command.angular.z * (std::cos(yaw) - std::cos(yaw + angle)) :
      command.linear.x * dt * std::sin(yaw);
    pose = worldPose(start.x + dx, start.y + dy, yaw + angle);
    updateTransform();
  }
  nav_msgs::msg::Path straight(double x = 4.0, double y = 0.0, double yaw = 0.0)
  {
    nav_msgs::msg::Path result;
    result.header.frame_id = "odom";
    for (int i = 0; i <= 80; ++i) {
      result.poses.push_back(worldPose(x * i / 80, y * i / 80, yaw));
    }
    return result;
  }
};

class PredictionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};
TEST_F(PredictionTest, StoppingHorizonAndEnvelope)
{
  PredictionHarness h;
  auto & c = h.controller;
  EXPECT_DOUBLE_EQ(c.predictionHorizon(velocity(0, 0, 0), velocity(0, 0, 0)), 3.0);
  EXPECT_DOUBLE_EQ(c.predictionHorizon(velocity(0, 0, 0), velocity(0.9, 0, 0)), 3.8);
  EXPECT_NEAR(c.stoppingDistance(velocity(0.5, 0, 0), 0.6), 0.25 + 0.5 / 30, 1e-12);
  const auto u = velocity(0.3, 0, 0.5);
  const double scale = c.brakingScale(u, 0.6, 0.03);
  EXPECT_NEAR(c.stoppingDistance(velocity(u.linear.x * scale, 0, u.angular.z * scale),
      0.6), 0.03, 1e-12);
  EXPECT_THROW(c.predictionHorizon(velocity(NAN, 0, 0), u), nav2_core::NoValidControl);
}
TEST_F(PredictionTest, RolloutMatchesControllerWithoutChangingState)
{
  PredictionHarness h;
  auto plan = h.straight();
  for (auto & p : plan.poses) {
    p.pose.position.y = 0.08 * std::sin(3 * p.pose.position.x);
  }
  auto & c = h.controller;
  c.setPlan(plan);
  auto feedback = velocity(0.3, 0, 0.04);
  c.seed(feedback);
  const auto first = c.firstStep(h.pose, feedback);
  const auto context = c.makePredictionContext(h.pose, feedback, feedback);
  const auto before_plan = c.plan();
  const auto before_flags = c.flags();
  const auto result = c.predictTracking(context, first);
  ASSERT_EQ(result.event, "clear");
  EXPECT_EQ(result.steps, 90u);
  EXPECT_EQ(c.plan(), before_plan);
  EXPECT_EQ(c.flags(), before_flags);
  EXPECT_EQ(c.command(), feedback);
  GoalChecker checker;
  for (size_t i = 0; i < result.commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_NEAR(h.pose.pose.position.x, result.control_poses[i].pose.position.x, 1e-9);
    EXPECT_NEAR(h.pose.pose.position.y, result.control_poses[i].pose.position.y, 1e-9);
    const auto actual = c.computeVelocityCommands(h.pose, feedback, &checker).twist;
    EXPECT_NEAR(actual.linear.x, result.commands[i].linear.x, 1e-9);
    EXPECT_NEAR(actual.angular.z, result.commands[i].angular.z, 1e-9);
    EXPECT_EQ(actual.linear.y, 0.0);
    EXPECT_LE(std::abs(actual.linear.x - feedback.linear.x), 0.5 / 30 + 1e-9);
    EXPECT_LE(std::abs(actual.angular.z - feedback.angular.z), 1.5 / 30 + 1e-9);
    feedback = actual;
    h.advance(actual);
  }
}
TEST_F(PredictionTest, SmallBendNearWallRejectsConstantArcButAcceptsTracking)
{
  PredictionHarness h;
  auto & c = h.controller;
  auto plan = h.straight();
  for (auto & p : plan.poses) {
    const double x = p.pose.position.x;
    p.pose.position.y = 0.08 * std::sin(3.0 * x);
  }
  c.setPlan(plan);
  const auto speed = velocity(0.5, 0, 0.2);
  c.seed(speed);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed);
  bool found = false;
  for (double x = 0.5; x < 1.4 && !found; x += 0.05) {
    for (double y = 0.32; y < 0.8 && !found; y += 0.025) {
      h.clear(); h.obstacle(x, y);
      if (c.constantCollision(h.pose, first.command) &&
        c.predictTracking(context, first).event == "clear")
      {
        found = true;
        EXPECT_NO_THROW(c.limitPredictedCollision(context, first));
      }
    }
  }
  EXPECT_TRUE(found);
}
TEST_F(PredictionTest, StraightAgreementAndObstacleDetection)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.setPlan(h.straight());
  const auto speed = velocity(0.5, 0, 0);
  c.seed(speed);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed);
  EXPECT_FALSE(c.constantCollision(h.pose, first.command));
  const auto clear = c.predictTracking(context, first);
  EXPECT_EQ(clear.event, "clear");
  for (size_t i = 0; i < clear.control_poses.size(); ++i) {
    EXPECT_NEAR(clear.control_poses[i].pose.position.x, 0.5 * i / 30, 1e-9);
    EXPECT_NEAR(clear.control_poses[i].pose.position.y, 0.0, 1e-12);
  }
  for (double y = -1; y <= 1; y += 0.025) {
    h.obstacle(0.9, y);
  }
  EXPECT_TRUE(c.constantCollision(h.pose, first.command));
  EXPECT_EQ(c.predictTracking(context, first).event, "collision");
}
TEST_F(PredictionTest, StepLimitAndImminentCollisionFailClosed)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.setPlan(h.straight());
  const auto speed = velocity(0.5, 0, 0);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed);
  c.maxSteps(89);
  const auto result = c.predictTracking(context, first);
  EXPECT_EQ(result.event, "step_limit");
  EXPECT_EQ(result.steps, 0u);
  EXPECT_THROW(c.limitPredictedCollision(context, first), nav2_core::NoValidControl);
  c.maxSteps(90);
  EXPECT_EQ(c.predictTracking(context, first).event, "clear");
  h.obstacle(0.5, 0);
  EXPECT_THROW(c.limitPredictedCollision(context, first), nav2_core::NoValidControl);
}
TEST_F(PredictionTest, StopsWithMarginWithinAccelerationLimitsAndResumes)
{
  PredictionHarness h;
  auto & c = h.controller;
  c.setPlan(h.straight());
  for (double y = -1; y <= 1; y += 0.025) {
    h.obstacle(1.4, y);
  }
  auto feedback = velocity(0.3, 0, 0);
  c.seed(feedback);
  c.enable(true);
  GoalChecker checker;
  int reductions = 0;
  for (int i = 0; i < 240; ++i) {
    SCOPED_TRACE(i);
    geometry_msgs::msg::Twist next;
    ASSERT_NO_THROW(next = c.computeVelocityCommands(h.pose, feedback, &checker).twist);
    EXPECT_LE(std::abs(next.linear.x - feedback.linear.x), 0.5 / 30 + 1e-9);
    EXPECT_LE(std::abs(next.angular.z - feedback.angular.z), 1.5 / 30 + 1e-9);
    if (next.linear.x < feedback.linear.x - 1e-7) {++reductions;}
    feedback = next;
    h.advance(feedback);
  }
  EXPECT_GT(reductions, 3);
  EXPECT_LT(std::abs(feedback.linear.x), 1e-6);
  EXPECT_LE(h.pose.pose.position.x + 0.42, 1.4 - 0.3 + 0.05);
  h.clear();
  EXPECT_GT(c.computeVelocityCommands(h.pose, feedback, &checker).twist.linear.x, 0.0);
}
TEST_F(PredictionTest, MapBoundaryIsNotUnverifiedFreeSpace)
{
  PredictionHarness h;
  h.map->getCostmap()->resizeMap(80, 80, 0.05, -2.7, -2.0);
  h.clear();
  auto & c = h.controller;
  c.setPlan(h.straight());
  const auto speed = velocity(0.3, 0, 0);
  const auto first = c.firstStep(h.pose, speed);
  const auto context = c.makePredictionContext(h.pose, speed, speed);
  EXPECT_EQ(c.predictTracking(context, first).event, "map_boundary");
  EXPECT_NO_THROW(c.limitPredictedCollision(context, first));
}
