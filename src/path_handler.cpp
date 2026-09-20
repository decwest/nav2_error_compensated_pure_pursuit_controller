// Copyright (c) 2022 Samsung Research America
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
// Modified to retain the continuous projection segment during path transformation.

#include "nav2_error_compensated_pure_pursuit_controller/path_handler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_error_compensated_pure_pursuit_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  tf2::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance),
  tf_(std::move(tf)),
  costmap_ros_(std::move(costmap_ros))
{
}

double PathHandler::getCostmapMaxExtent() const
{
  const double maximum_dimension = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return maximum_dimension / 2.0;
}

PathHandler::TransformedPlan PathHandler::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  const double max_robot_pose_search_dist,
  const bool reject_unit_path)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }
  if (reject_unit_path && global_plan_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan frame");
  }

  // Choose the nearest continuous segment rather than the nearest sampled vertex. This preserves
  // sparse/dense sampling equivalence and prevents a nearby vertex on a later branch from jumping
  // progress past an earlier segment containing the robot projection. Exact-distance ties remain
  // on the earlier path segment by projectPointToPath's strict comparison.
  const auto global_projection = arc_length_lookahead::projectPointToPath(
    global_plan_, robot_pose.pose.position,
    std::max(max_robot_pose_search_dist, std::numeric_limits<double>::epsilon()));

  auto transformation_begin = global_plan_.poses.begin();
  if (global_projection.valid) {
    transformation_begin = std::next(
      global_plan_.poses.begin(), static_cast<std::ptrdiff_t>(global_projection.segment_index));
  }

  // Always include both endpoints of the selected projection segment. Costmap clipping begins at
  // the following vertex, so an outside predecessor cannot make the transformed plan empty.
  auto clipping_begin = transformation_begin;
  if (clipping_begin != global_plan_.poses.end()) {
    clipping_begin = std::next(clipping_begin);
  }
  if (clipping_begin != global_plan_.poses.end()) {
    clipping_begin = std::next(clipping_begin);
  }
  const double maximum_costmap_extent = getCostmapMaxExtent();
  const auto transformation_end = std::find_if(
    clipping_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > maximum_costmap_extent;
    });

  const auto transform_to_local = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose;
      geometry_msgs::msg::PoseStamped transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
        throw nav2_core::ControllerTFError("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  TransformedPlan transformed;
  auto & transformed_plan = transformed.path;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses), transform_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Prune only vertices preceding the retained predecessor.
  global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it");
  }

  // Keep the projection selected within max_robot_pose_search_dist. Searching the transformed
  // path again could choose a later branch outside that bound. The selected segment is now first.
  if (global_projection.valid) {
    constexpr double kMinSegmentLength = 1e-9;
    const auto & start = transformed_plan.poses[0].pose.position;
    const auto & end = transformed_plan.poses[1].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length > kMinSegmentLength) {
      auto & projection = transformed.projection;
      projection.valid = true;
      projection.interpolation_ratio = global_projection.interpolation_ratio;
      projection.position.x = start.x + projection.interpolation_ratio * dx;
      projection.position.y = start.y + projection.interpolation_ratio * dy;
      projection.tangent_x = dx / segment_length;
      projection.tangent_y = dy / segment_length;
      projection.arc_length = projection.interpolation_ratio * segment_length;

      // Arc lengths refer to the retained, costmap-clipped path, not to the full global plan.
      for (size_t index = 0; index + 1 < transformed_plan.poses.size(); ++index) {
        const auto & p0 = transformed_plan.poses[index].pose.position;
        const auto & p1 = transformed_plan.poses[index + 1].pose.position;
        const double length = std::hypot(p1.x - p0.x, p1.y - p0.y);
        if (length > kMinSegmentLength) {
          projection.path_length += length;
        }
      }
      projection.remaining_arc_length = std::max(
        projection.path_length - projection.arc_length, 0.0);
    }
  }
  return transformed;
}

bool PathHandler::transformPose(
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", exception.what());
  }
  return false;
}

}  // namespace nav2_error_compensated_pure_pursuit_controller
