// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_error_compensated_pure_pursuit_controller
{

/**
 * @brief Plugin-local global-plan transformer that retains the projection segment predecessor.
 *
 * Navigation2's RPP PathHandler begins its transformed path at the nearest sampled path vertex.
 * This implementation instead finds the nearest continuous non-zero segment within the progress
 * search bound and starts transformation/pruning at that segment's front vertex. Sparse and dense
 * sampling therefore make the same progress choice, with exact ties resolved toward earlier path
 * progress. Navigation2 itself is not modified.
 */
class PathHandler
{
public:
  PathHandler(
    tf2::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double max_robot_pose_search_dist,
    bool reject_unit_path = false);

  bool transformPose(
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  void setPlan(const nav_msgs::msg::Path & path) {global_plan_ = path;}

  nav_msgs::msg::Path getPlan() const {return global_plan_;}

private:
  double getCostmapMaxExtent() const;

  rclcpp::Logger logger_{rclcpp::get_logger("ECPPPathHandler")};
  tf2::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace nav2_error_compensated_pure_pursuit_controller

#endif  // NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
