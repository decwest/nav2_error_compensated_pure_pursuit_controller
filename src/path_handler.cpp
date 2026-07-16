// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include "nav2_error_compensated_pure_pursuit_controller/path_handler.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"
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

nav_msgs::msg::Path PathHandler::transformGlobalPlan(
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

  nav_msgs::msg::Path transformed_plan;
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
  return transformed_plan;
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
