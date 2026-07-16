// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ARC_LENGTH_LOOKAHEAD_HPP_
#define NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ARC_LENGTH_LOOKAHEAD_HPP_

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav_msgs/msg/path.hpp"

namespace nav2_error_compensated_pure_pursuit_controller
{
namespace arc_length_lookahead
{

/**
 * @brief Continuous projection of the robot origin onto a transformed polyline path.
 *
 * The transformed path is expressed in the robot frame, hence the robot pose is (0, 0, 0).
 * Arc lengths are measured from the beginning of the supplied path. Pose orientations are not
 * used: the path tangent is always derived from the selected non-zero segment.
 */
struct PathProjection
{
  bool valid{false};
  size_t segment_index{0};
  double interpolation_ratio{0.0};
  double arc_length{0.0};
  double path_length{0.0};
  double remaining_arc_length{0.0};
  geometry_msgs::msg::Point position;
  double tangent_x{1.0};
  double tangent_y{0.0};
};

/**
 * @brief Project an arbitrary query point onto the nearest usable polyline segment.
 *
 * Candidate segments start within @p search_window_m of the beginning of the retained path.
 * The full first sparse segment is still considered even when it is longer than the window.
 * Equal-distance ties select the earlier path segment, providing deterministic forward progress
 * when the retained path crosses itself. Zero-length segments are ignored.
 */
inline PathProjection projectPointToPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & query_point,
  const double search_window_m = std::numeric_limits<double>::infinity())
{
  constexpr double kMinSegmentLength = 1e-9;
  if (std::isnan(search_window_m) || search_window_m <= 0.0) {
    throw std::invalid_argument("Path projection search window must be positive");
  }
  if (!std::isfinite(query_point.x) || !std::isfinite(query_point.y)) {
    throw std::invalid_argument("Path projection query point must be finite");
  }

  PathProjection projection;
  double nearest_distance_squared = std::numeric_limits<double>::max();
  double path_arc_length = 0.0;

  for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto & start = path.poses[index].pose.position;
    const auto & end = path.poses[index + 1].pose.position;
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_length_squared =
      segment_x * segment_x + segment_y * segment_y;
    const double segment_length = std::sqrt(segment_length_squared);

    if (segment_length <= kMinSegmentLength) {
      continue;
    }

    // Evaluate the entire first/sparse segment whose start is within the search window. This is
    // essential when the robot lies inside a long segment and its predecessor vertex is far away.
    if (path_arc_length <= search_window_m) {
      const double projection_ratio = std::clamp(
        ((query_point.x - start.x) * segment_x +
        (query_point.y - start.y) * segment_y) / segment_length_squared,
        0.0, 1.0);
      const double projection_x = start.x + projection_ratio * segment_x;
      const double projection_y = start.y + projection_ratio * segment_y;
      const double projection_distance_squared =
        (projection_x - query_point.x) * (projection_x - query_point.x) +
        (projection_y - query_point.y) * (projection_y - query_point.y);

      // Strict comparison deliberately keeps the earlier candidate on an exact-distance tie.
      if (projection_distance_squared < nearest_distance_squared) {
        nearest_distance_squared = projection_distance_squared;
        projection.valid = true;
        projection.segment_index = index;
        projection.interpolation_ratio = projection_ratio;
        projection.arc_length = path_arc_length + projection_ratio * segment_length;
        projection.position.x = projection_x;
        projection.position.y = projection_y;
        projection.position.z =
          start.z + projection_ratio * (end.z - start.z);
        projection.tangent_x = segment_x / segment_length;
        projection.tangent_y = segment_y / segment_length;
      }
    }

    path_arc_length += segment_length;
  }

  projection.path_length = path_arc_length;
  if (projection.valid) {
    projection.remaining_arc_length = std::max(
      projection.path_length - projection.arc_length, 0.0);
  }
  return projection;
}

inline PathProjection projectPath(
  const nav_msgs::msg::Path & path,
  const double search_window_m = std::numeric_limits<double>::infinity())
{
  geometry_msgs::msg::Point robot_origin;
  return projectPointToPath(path, robot_origin, search_window_m);
}

namespace detail
{

inline geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  const double interpolation_ratio)
{
  if (interpolation_ratio <= 0.0) {
    return start;
  }
  if (interpolation_ratio >= 1.0) {
    return end;
  }

  auto interpolated_pose = end;
  interpolated_pose.pose.position.x =
    start.pose.position.x +
    interpolation_ratio * (end.pose.position.x - start.pose.position.x);
  interpolated_pose.pose.position.y =
    start.pose.position.y +
    interpolation_ratio * (end.pose.position.y - start.pose.position.y);
  interpolated_pose.pose.position.z =
    start.pose.position.z +
    interpolation_ratio * (end.pose.position.z - start.pose.position.z);
  return interpolated_pose;
}

}  // namespace detail

/**
 * @brief Find the path point a given arc length ahead of the robot's nearest projection.
 *
 * The path is expressed in the robot frame, so the robot is at the origin. Both the nearest
 * projection and the lookahead point are continuously interpolated between path poses. A target
 * beyond the available path is clamped to the final pose and is therefore always on the path.
 *
 * @param lookahead_dist Arc length to advance from the nearest projection
 * @param path Path expressed in the robot frame
 * @return Interpolated lookahead pose on the path
 */
inline geometry_msgs::msg::PoseStamped getLookAheadPoint(
  const double lookahead_dist,
  const nav_msgs::msg::Path & path,
  const PathProjection & projection)
{
  constexpr double kMinSegmentLength = 1e-9;

  if (path.poses.empty()) {
    throw std::invalid_argument("Cannot compute an arc-length lookahead on an empty path");
  }
  if (!std::isfinite(lookahead_dist) || lookahead_dist < 0.0) {
    throw std::invalid_argument("Arc-length lookahead distance must be finite and non-negative");
  }
  if (path.poses.size() == 1) {
    return path.poses.front();
  }
  if (!projection.valid || projection.path_length <= kMinSegmentLength) {
    return path.poses.back();
  }

  const double target_arc_length = std::min(
    projection.arc_length + lookahead_dist, projection.path_length);
  if (target_arc_length >= projection.path_length) {
    return path.poses.back();
  }
  double segment_start_arc_length = 0.0;

  for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto & start = path.poses[index].pose.position;
    const auto & end = path.poses[index + 1].pose.position;
    const double segment_length = std::hypot(end.x - start.x, end.y - start.y);

    if (segment_length <= kMinSegmentLength) {
      continue;
    }

    const double segment_end_arc_length = segment_start_arc_length + segment_length;
    if (target_arc_length <= segment_end_arc_length) {
      const double interpolation_ratio = std::clamp(
        (target_arc_length - segment_start_arc_length) / segment_length,
        0.0, 1.0);
      return detail::interpolatePose(
        path.poses[index], path.poses[index + 1], interpolation_ratio);
    }

    segment_start_arc_length = segment_end_arc_length;
  }

  return path.poses.back();
}

/**
 * @brief Convenience overload that computes a projection for a single standalone query.
 *
 * A controller cycle should call projectPath() once and use the explicit-projection overload so
 * carrot, tracking error, and cusp computations share exactly the same progress point.
 */
inline geometry_msgs::msg::PoseStamped getLookAheadPoint(
  const double lookahead_dist,
  const nav_msgs::msg::Path & path)
{
  return getLookAheadPoint(lookahead_dist, path, projectPath(path));
}

/**
 * @brief Find the path arc length from the nearest projection to the next velocity cusp.
 *
 * A cusp is detected by a sign reversal between consecutive non-zero path segments. Zero-length
 * segments and all stored pose orientations are ignored. Returning the maximum double when no
 * cusp exists matches RPP's caller-facing behavior.
 *
 * @param path Path expressed in the robot frame
 * @return Arc length from the nearest projection to the next cusp
 */
inline double findVelocitySignChangeArcLength(
  const nav_msgs::msg::Path & path,
  const PathProjection & projection)
{
  constexpr double kMinSegmentLength = 1e-9;
  if (path.poses.size() < 3 || !projection.valid) {
    return std::numeric_limits<double>::max();
  }

  double segment_start_arc_length = 0.0;
  double previous_segment_x = 0.0;
  double previous_segment_y = 0.0;
  bool has_previous_nonzero_segment = false;

  for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto & start = path.poses[index].pose.position;
    const auto & end = path.poses[index + 1].pose.position;
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_length = std::hypot(segment_x, segment_y);
    if (segment_length <= kMinSegmentLength) {
      continue;
    }

    const bool direction_changes = has_previous_nonzero_segment &&
      previous_segment_x * segment_x + previous_segment_y * segment_y < 0.0;
    if (direction_changes && segment_start_arc_length >= projection.arc_length) {
      return segment_start_arc_length - projection.arc_length;
    }

    previous_segment_x = segment_x;
    previous_segment_y = segment_y;
    has_previous_nonzero_segment = true;
    segment_start_arc_length += segment_length;
  }

  return std::numeric_limits<double>::max();
}

inline double findVelocitySignChangeArcLength(const nav_msgs::msg::Path & path)
{
  return findVelocitySignChangeArcLength(path, projectPath(path));
}

}  // namespace arc_length_lookahead
}  // namespace nav2_error_compensated_pure_pursuit_controller

#endif  // NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ARC_LENGTH_LOOKAHEAD_HPP_
