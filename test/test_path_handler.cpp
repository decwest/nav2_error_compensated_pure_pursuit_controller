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
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/ecpp_math.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/path_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ecpp_controller = nav2_error_compensated_pure_pursuit_controller;
namespace arc_lookahead = ecpp_controller::arc_length_lookahead;

namespace
{

class RclcppFixture
{
public:
  RclcppFixture() {rclcpp::init(0, nullptr);}
  ~RclcppFixture() {rclcpp::shutdown();}
};

RclcppFixture g_rclcpp_fixture;

nav_msgs::msg::Path makePath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = stamp;
  for (const double x : {-0.9, 0.2, 1.0, 1.8}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path makePath(
  const rclcpp::Time & stamp,
  const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = stamp;
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(EcppPathHandler, RetainsProjectionSegmentPredecessorAcrossPruning)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ecpp_path_handler_test");
  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("ecpp_path_handler_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  ecpp_controller::PathHandler handler(
    tf2::durationFromSec(0.1), transform_buffer, costmap);
  const auto stamp = node->get_clock()->now();
  handler.setPlan(makePath(stamp));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;

  // The nearest vertex is x=0.2. The x=-0.9 predecessor is required to project the origin onto
  // the containing segment; starting at x=0.2 would incorrectly advance progress by 0.2 m.
  const auto transformed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(transformed.path.poses.size(), 2u);
  EXPECT_NEAR(transformed.path.poses.front().pose.position.x, -0.9, 1e-12);
  const auto & projection = transformed.projection;
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.position.x, 0.0, 1e-12);
  EXPECT_NEAR(projection.arc_length, 0.9, 1e-12);

  const auto retained_plan = handler.getPlan();
  ASSERT_FALSE(retained_plan.poses.empty());
  EXPECT_NEAR(retained_plan.poses.front().pose.position.x, -0.9, 1e-12);

  // A second cycle makes the same progress choice, demonstrating that pruning did not discard
  // the active segment predecessor.
  const auto transformed_again = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(transformed_again.path.poses.size(), 2u);
  EXPECT_NEAR(transformed_again.path.poses.front().pose.position.x, -0.9, 1e-12);
  ASSERT_TRUE(transformed_again.projection.valid);
  EXPECT_NEAR(transformed_again.projection.arc_length, 0.9, 1e-12);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(EcppPathHandler, SearchBoundIsPreservedForErrorsAndLookahead)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ecpp_search_bound_test");
  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("ecpp_search_bound_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  ecpp_controller::PathHandler handler(
    tf2::durationFromSec(0.1), transform_buffer, costmap);
  const auto stamp = node->get_clock()->now();
  handler.setPlan(makePath(
      stamp, {{-0.5, 0.3}, {0.5, 0.3}, {0.5, 0.05}, {-0.5, 0.05}}));
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;

  // Only the first segment starts within the bound. It is considered in full even though
  // the robot's projection lies farther along it than the bound itself.
  const auto transformed = handler.transformGlobalPlan(robot_pose, 0.1);
  ASSERT_EQ(transformed.path.poses.size(), 4u);
  const auto & projection = transformed.projection;
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.position.x, 0.0, 1e-12);
  EXPECT_NEAR(projection.position.y, 0.3, 1e-12);
  EXPECT_NEAR(projection.arc_length, 0.5, 1e-12);
  EXPECT_NEAR(projection.path_length, 2.25, 1e-12);
  EXPECT_NEAR(projection.remaining_arc_length, 1.75, 1e-12);

  // A separate search over the transformed path would jump to the closer returning branch.
  const auto later_projection = arc_lookahead::projectPath(transformed.path);
  EXPECT_EQ(later_projection.segment_index, 2u);
  EXPECT_NEAR(later_projection.position.y, 0.05, 1e-12);

  const auto error = ecpp_controller::ecpp_math::computePathFrameError(projection);
  EXPECT_NEAR(error.e_y, -0.3, 1e-12);
  EXPECT_NEAR(error.e_psi, 0.0, 1e-12);
  const auto carrot = arc_lookahead::getLookAheadPoint(0.4, transformed.path, projection);
  EXPECT_NEAR(carrot.pose.position.x, 0.4, 1e-12);
  EXPECT_NEAR(carrot.pose.position.y, 0.3, 1e-12);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(EcppPathHandler, ProjectionTransformsAndArcLengthsFollowClipping)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ecpp_projection_tf_test");
  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("ecpp_projection_tf_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  const auto stamp = node->get_clock()->now();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = stamp;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.2;
  transform.transform.rotation.z = std::sin(M_PI / 4.0);
  transform.transform.rotation.w = std::cos(M_PI / 4.0);
  ASSERT_TRUE(transform_buffer->setTransform(transform, "test", true));

  auto path = makePath(
    stamp, {{-4.0, 2.0}, {0.0, 2.0}, {3.0, 2.0}, {2.0, 2.0}, {10.0, 2.0}});
  path.header.frame_id = "map";
  for (auto & pose : path.poses) {
    pose.header = path.header;
  }
  ecpp_controller::PathHandler handler(
    tf2::durationFromSec(0.1), transform_buffer, costmap);
  handler.setPlan(path);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = transform.header;
  robot_pose.pose.position.x = 1.0;
  robot_pose.pose.position.y = 2.2;
  robot_pose.pose.orientation = transform.transform.rotation;
  const auto transformed = handler.transformGlobalPlan(robot_pose, 10.0);
  EXPECT_EQ(transformed.path.header.frame_id, "base_link");
  ASSERT_EQ(transformed.path.poses.size(), 3u);
  const auto & projection = transformed.projection;
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.interpolation_ratio, 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(projection.position.x, -0.2, 1e-12);
  EXPECT_NEAR(projection.position.y, 0.0, 1e-12);
  EXPECT_NEAR(projection.tangent_x, 0.0, 1e-12);
  EXPECT_NEAR(projection.tangent_y, -1.0, 1e-12);
  EXPECT_NEAR(projection.arc_length, 1.0, 1e-12);
  EXPECT_NEAR(projection.path_length, 4.0, 1e-12);
  EXPECT_NEAR(projection.remaining_arc_length, 3.0, 1e-12);

  const auto error = ecpp_controller::ecpp_math::computePathFrameError(projection);
  EXPECT_NEAR(error.e_y, 0.2, 1e-12);
  EXPECT_NEAR(error.e_psi, M_PI / 2.0, 1e-12);
  const auto carrot = arc_lookahead::getLookAheadPoint(0.5, transformed.path, projection);
  EXPECT_NEAR(carrot.pose.position.x, -0.2, 1e-12);
  EXPECT_NEAR(carrot.pose.position.y, -0.5, 1e-12);
  const auto endpoint = arc_lookahead::getLookAheadPoint(10.0, transformed.path, projection);
  EXPECT_NEAR(endpoint.pose.position.x, -0.2, 1e-12);
  EXPECT_NEAR(endpoint.pose.position.y, -1.0, 1e-12);
  EXPECT_NEAR(arc_lookahead::findVelocitySignChangeArcLength(transformed.path, projection),
    2.0, 1e-12);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(EcppPathHandler, DegeneratePlansHaveNoProjectionAndKeepEndpointFallback)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ecpp_degenerate_plan_test");
  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("ecpp_degenerate_plan_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  ecpp_controller::PathHandler handler(
    tf2::durationFromSec(0.1), transform_buffer, costmap);
  const auto stamp = node->get_clock()->now();
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;

  for (const auto & path : {
    makePath(stamp, {{0.1, 0.2}}), makePath(stamp, {{0.1, 0.2}, {0.1, 0.2}})})
  {
    handler.setPlan(path);
    const auto transformed = handler.transformGlobalPlan(robot_pose, 2.0);
    EXPECT_FALSE(transformed.projection.valid);
    EXPECT_FALSE(ecpp_controller::ecpp_math::computePathFrameError(transformed.projection).valid);
    const auto carrot = arc_lookahead::getLookAheadPoint(
      1.0, transformed.path, transformed.projection);
    EXPECT_NEAR(carrot.pose.position.x, 0.1, 1e-12);
    EXPECT_NEAR(carrot.pose.position.y, 0.2, 1e-12);
  }

  costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(EcppPathHandler, ContinuousSegmentSelectionBeatsNearbyFutureVertex)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ecpp_sparse_handler_test");
  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("ecpp_sparse_handler_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  ecpp_controller::PathHandler handler(
    tf2::durationFromSec(0.1), transform_buffer, costmap);
  const auto stamp = node->get_clock()->now();
  handler.setPlan(makePath(
      stamp, {{-2.0, 0.1}, {2.0, 0.1}, {2.0, 2.0}, {0.0, 0.2}}));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;

  // The future vertex (0, 0.2) is the nearest sampled vertex, but the first sparse segment is
  // only 0.1 m from the robot. Continuous selection must retain that first segment.
  const auto transformed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(transformed.path.poses.size(), 2u);
  EXPECT_NEAR(transformed.path.poses.front().pose.position.x, -2.0, 1e-12);
  EXPECT_NEAR(transformed.path.poses.front().pose.position.y, 0.1, 1e-12);
  const auto & projection = transformed.projection;
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.position.x, 0.0, 1e-12);
  EXPECT_NEAR(projection.position.y, 0.1, 1e-12);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}

TEST(EcppPathHandler, SelfIntersectionTieRetainsEarlierProgressSegment)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ecpp_crossing_handler_test");
  auto transform_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("ecpp_crossing_handler_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  ecpp_controller::PathHandler handler(
    tf2::durationFromSec(0.1), transform_buffer, costmap);
  const auto stamp = node->get_clock()->now();
  handler.setPlan(makePath(
      stamp,
      {{-1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0, 0.0}, {1.0, 0.0}}));

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = stamp;
  robot_pose.pose.orientation.w = 1.0;
  const auto transformed = handler.transformGlobalPlan(robot_pose, 10.0);

  ASSERT_GE(transformed.path.poses.size(), 2u);
  EXPECT_NEAR(transformed.path.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(transformed.path.poses.front().pose.position.y, 0.0, 1e-12);
  const auto & projection = transformed.projection;
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.arc_length, 1.0, 1e-12);

  // Advance around the loop so pruning removes the first crossing branch.
  robot_pose.pose.position.x = 1.0;
  robot_pose.pose.position.y = 0.8;
  auto progressed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(progressed.path.poses.size(), 2u);
  EXPECT_NEAR(progressed.path.poses.front().pose.position.x, 1.0, 1e-12);
  EXPECT_NEAR(progressed.path.poses.front().pose.position.y, 0.0, 1e-12);

  robot_pose.pose.position.x = -1.0;
  progressed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(progressed.path.poses.size(), 2u);
  EXPECT_NEAR(progressed.path.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(progressed.path.poses.front().pose.position.y, 1.0, 1e-12);

  // Returning to the geometric crossing now selects the retained later branch, rather than
  // jumping backward to the first branch that pruning deliberately removed.
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  progressed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(progressed.path.poses.size(), 2u);
  EXPECT_NEAR(progressed.path.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(progressed.path.poses.front().pose.position.y, 0.0, 1e-12);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}
