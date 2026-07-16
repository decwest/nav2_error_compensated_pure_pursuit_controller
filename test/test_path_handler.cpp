// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/path_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ecpp_controller = nav2_error_compensated_pure_pursuit_controller;

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
  ASSERT_GE(transformed.poses.size(), 2u);
  EXPECT_NEAR(transformed.poses.front().pose.position.x, -0.9, 1e-12);
  const auto projection = ecpp_controller::arc_length_lookahead::projectPath(transformed);
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
  ASSERT_GE(transformed_again.poses.size(), 2u);
  EXPECT_NEAR(transformed_again.poses.front().pose.position.x, -0.9, 1e-12);

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
  ASSERT_GE(transformed.poses.size(), 2u);
  EXPECT_NEAR(transformed.poses.front().pose.position.x, -2.0, 1e-12);
  EXPECT_NEAR(transformed.poses.front().pose.position.y, 0.1, 1e-12);
  const auto projection = ecpp_controller::arc_length_lookahead::projectPath(transformed);
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

  ASSERT_GE(transformed.poses.size(), 2u);
  EXPECT_NEAR(transformed.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(transformed.poses.front().pose.position.y, 0.0, 1e-12);
  const auto projection = ecpp_controller::arc_length_lookahead::projectPath(transformed);
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.arc_length, 1.0, 1e-12);

  // Advance around the loop so pruning removes the first crossing branch.
  robot_pose.pose.position.x = 1.0;
  robot_pose.pose.position.y = 0.8;
  auto progressed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(progressed.poses.size(), 2u);
  EXPECT_NEAR(progressed.poses.front().pose.position.x, 1.0, 1e-12);
  EXPECT_NEAR(progressed.poses.front().pose.position.y, 0.0, 1e-12);

  robot_pose.pose.position.x = -1.0;
  progressed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(progressed.poses.size(), 2u);
  EXPECT_NEAR(progressed.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(progressed.poses.front().pose.position.y, 1.0, 1e-12);

  // Returning to the geometric crossing now selects the retained later branch, rather than
  // jumping backward to the first branch that pruning deliberately removed.
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  progressed = handler.transformGlobalPlan(robot_pose, 10.0);
  ASSERT_GE(progressed.poses.size(), 2u);
  EXPECT_NEAR(progressed.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(progressed.poses.front().pose.position.y, 0.0, 1e-12);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}
