// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"

namespace arc_length_lookahead =
  nav2_error_compensated_pure_pursuit_controller::arc_length_lookahead;

namespace
{

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
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

void expectPositionNear(
  const geometry_msgs::msg::PoseStamped & pose,
  const double expected_x,
  const double expected_y,
  const double tolerance = 1e-9)
{
  EXPECT_NEAR(pose.pose.position.x, expected_x, tolerance);
  EXPECT_NEAR(pose.pose.position.y, expected_y, tolerance);
}

}  // namespace

TEST(ArcLengthLookahead, RejectsInvalidInput)
{
  EXPECT_THROW(
    arc_length_lookahead::getLookAheadPoint(0.5, makePath({})),
    std::invalid_argument);
  EXPECT_THROW(
    arc_length_lookahead::getLookAheadPoint(-0.1, makePath({{0.0, 0.0}})),
    std::invalid_argument);
  EXPECT_THROW(
    arc_length_lookahead::getLookAheadPoint(
      std::numeric_limits<double>::quiet_NaN(), makePath({{0.0, 0.0}})),
    std::invalid_argument);
}

TEST(ArcLengthLookahead, SinglePoseReturnsItself)
{
  const auto path = makePath({{1.2, -0.4}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(10.0, path);

  expectPositionNear(lookahead, 1.2, -0.4);
  EXPECT_EQ(lookahead.header.frame_id, "base_link");
}

TEST(ArcLengthLookahead, TwoPosePathProjectsAndInterpolates)
{
  const auto path = makePath({{-1.0, 0.2}, {2.0, 0.2}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(0.5, path);

  expectPositionNear(lookahead, 0.5, 0.2);
}

TEST(ArcLengthLookahead, ProjectionReportsSegmentRatioArcTangentAndRemainingLength)
{
  const auto path = makePath({{-2.0, 0.3}, {1.0, 0.3}, {1.0, 2.3}});
  const auto projection = arc_length_lookahead::projectPath(path);

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.interpolation_ratio, 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(projection.arc_length, 2.0, 1e-12);
  EXPECT_NEAR(projection.position.x, 0.0, 1e-12);
  EXPECT_NEAR(projection.position.y, 0.3, 1e-12);
  EXPECT_NEAR(projection.tangent_x, 1.0, 1e-12);
  EXPECT_NEAR(projection.tangent_y, 0.0, 1e-12);
  EXPECT_NEAR(projection.path_length, 5.0, 1e-12);
  EXPECT_NEAR(projection.remaining_arc_length, 3.0, 1e-12);
}

TEST(ArcLengthLookahead, ProjectsArbitraryQueryPointForGlobalPathProgress)
{
  const auto path = makePath({{-1.0, 0.0}, {2.0, 0.0}});
  geometry_msgs::msg::Point query;
  query.x = 0.4;
  query.y = 0.3;
  const auto projection = arc_length_lookahead::projectPointToPath(path, query);

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.interpolation_ratio, 1.4 / 3.0, 1e-12);
  EXPECT_NEAR(projection.arc_length, 1.4, 1e-12);
  EXPECT_NEAR(projection.position.x, 0.4, 1e-12);
  EXPECT_NEAR(projection.position.y, 0.0, 1e-12);
}

TEST(ArcLengthLookahead, EqualDistanceTieSelectsEarlierPathProgress)
{
  // Both horizontal segments pass through the origin. The earlier segment must win so a
  // self-intersection cannot jump forward merely because it is sampled later in the path.
  const auto path = makePath(
    {{-1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0, 0.0}, {1.0, 0.0}});
  const auto projection = arc_length_lookahead::projectPath(path);

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.arc_length, 1.0, 1e-12);
}

TEST(ArcLengthLookahead, ExactCornerUsesIncomingSegmentAndOutgoingArcForCarrot)
{
  const auto path = makePath({{-1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}});
  const auto projection = arc_length_lookahead::projectPath(path);
  const auto carrot = arc_length_lookahead::getLookAheadPoint(0.25, path, projection);

  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.interpolation_ratio, 1.0, 1e-12);
  EXPECT_NEAR(projection.tangent_x, 1.0, 1e-12);
  EXPECT_NEAR(projection.tangent_y, 0.0, 1e-12);
  expectPositionNear(carrot, 0.0, 0.25);
}

TEST(ArcLengthLookahead, StraightPathMatchesCircleIntersectionOnPath)
{
  const auto path = makePath({{-1.0, 0.0}, {2.0, 0.0}});
  const double lookahead_dist = 0.6;
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(lookahead_dist, path);

  expectPositionNear(lookahead, lookahead_dist, 0.0);
  EXPECT_NEAR(
    std::hypot(lookahead.pose.position.x, lookahead.pose.position.y),
    lookahead_dist, 1e-9);
}

TEST(ArcLengthLookahead, AdvancesAcrossMultipleSegments)
{
  const auto path = makePath({{-1.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(1.5, path);

  expectPositionNear(lookahead, 1.0, 0.5);
}

TEST(ArcLengthLookahead, IsDefinedWhenLateralErrorExceedsLookahead)
{
  const auto path = makePath({{-1.0, 2.0}, {2.0, 2.0}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(0.5, path);

  expectPositionNear(lookahead, 0.5, 2.0);
  EXPECT_GT(std::hypot(lookahead.pose.position.x, lookahead.pose.position.y), 0.5);
}

TEST(ArcLengthLookahead, LookaheadPastPathEndReturnsFinalPose)
{
  const auto path = makePath({{-1.0, 0.2}, {2.0, 0.2}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(10.0, path);

  expectPositionNear(lookahead, 2.0, 0.2);
}

TEST(ArcLengthLookahead, SparseAndDensePathsGiveSameLookahead)
{
  const auto sparse_path = makePath({{-1.0, 0.2}, {2.0, 0.2}});
  std::vector<std::pair<double, double>> dense_points;
  for (int index = -10; index <= 20; ++index) {
    dense_points.emplace_back(0.1 * index, 0.2);
  }
  const auto dense_path = makePath(dense_points);

  const auto sparse_lookahead = arc_length_lookahead::getLookAheadPoint(0.73, sparse_path);
  const auto dense_lookahead = arc_length_lookahead::getLookAheadPoint(0.73, dense_path);

  expectPositionNear(sparse_lookahead, 0.73, 0.2);
  expectPositionNear(dense_lookahead, 0.73, 0.2);
}

TEST(ArcLengthLookahead, ZeroLengthSegmentsAreSkipped)
{
  const auto path = makePath(
    {{-1.0, 0.0}, {-1.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(1.5, path);

  expectPositionNear(lookahead, 1.5, 0.0);
}

TEST(ArcLengthLookahead, AllZeroLengthSegmentsReturnFinalPose)
{
  auto path = makePath({{0.3, -0.2}, {0.3, -0.2}, {0.3, -0.2}});
  path.poses.back().header.stamp.sec = 42;
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(0.5, path);

  expectPositionNear(lookahead, 0.3, -0.2);
  EXPECT_EQ(lookahead.header.stamp.sec, 42);
}

TEST(ArcLengthLookahead, ZeroLookaheadReturnsNearestProjection)
{
  const auto path = makePath({{-1.0, 0.4}, {2.0, 0.4}});
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(0.0, path);

  expectPositionNear(lookahead, 0.0, 0.4);
}

TEST(ArcLengthLookahead, InterpolationIsContinuousAcrossSegmentBoundary)
{
  const auto path = makePath({{-1.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}});
  constexpr double kDelta = 1e-6;
  const auto before_vertex = arc_length_lookahead::getLookAheadPoint(1.0 - kDelta, path);
  const auto after_vertex = arc_length_lookahead::getLookAheadPoint(1.0 + kDelta, path);

  const double displacement = std::hypot(
    after_vertex.pose.position.x - before_vertex.pose.position.x,
    after_vertex.pose.position.y - before_vertex.pose.position.y);
  EXPECT_LT(displacement, 3.0 * kDelta);
}

TEST(ArcLengthLookahead, CuspLimitUsesPathArcLengthFromNearestProjection)
{
  // The next direction reversal is only 0.5 m along the path, while its Euclidean distance from
  // the robot is hypot(0.5, 2.0). Using that chord as an arc-length query would cross the cusp.
  const auto path = makePath({{-1.0, 2.0}, {0.0, 2.0}, {0.5, 2.0}, {-2.0, 2.0}});
  const double distance_to_cusp =
    arc_length_lookahead::findVelocitySignChangeArcLength(path);
  const auto lookahead = arc_length_lookahead::getLookAheadPoint(distance_to_cusp, path);

  EXPECT_NEAR(distance_to_cusp, 0.5, 1e-9);
  expectPositionNear(lookahead, 0.5, 2.0);
}

TEST(ArcLengthLookahead, CuspSearchIgnoresOrientationAndZeroLengthSegments)
{
  const auto straight_path = makePath({{-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}});
  EXPECT_EQ(
    arc_length_lookahead::findVelocitySignChangeArcLength(straight_path),
    std::numeric_limits<double>::max());

  auto rotation_path = makePath({{-1.0, 0.0}, {0.5, 0.0}, {0.5, 0.0}, {1.0, 0.0}});
  rotation_path.poses[2].pose.orientation.z = 1.0;
  rotation_path.poses[2].pose.orientation.w = 0.0;
  EXPECT_EQ(
    arc_length_lookahead::findVelocitySignChangeArcLength(rotation_path),
    std::numeric_limits<double>::max());

  // A direction reversal remains detectable when duplicate poses separate its non-zero segments.
  const auto reversal_with_duplicates = makePath(
    {{-1.0, 0.0}, {0.5, 0.0}, {0.5, 0.0}, {0.5, 0.0}, {-1.0, 0.0}});
  EXPECT_NEAR(
    arc_length_lookahead::findVelocitySignChangeArcLength(reversal_with_duplicates),
    0.5, 1e-9);
}
